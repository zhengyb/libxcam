/*
 * test-surround-view.cpp - test surround view
 *
 *  Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Yinhang Liu <yinhangx.liu@intel.com>
 */

#include "test_common.h"
#include "test_stream.h"
#include "test_sv_params.h"
#include <interface/geo_mapper.h>
#include <interface/stitcher.h>
#include <calibration_parser.h>
#include <fisheye_image_file.h>
#include <soft/soft_video_buf_allocator.h>
#include <dma_video_buffer.h>
#if HAVE_GLES
#include <gles/gl_video_buffer.h>
#include <gles/egl/egl_base.h>
#include <gles/gl_texture.h>
#endif
#if HAVE_VULKAN
#include <vulkan/vk_device.h>
#endif

using namespace XCam;

/*
 * 本测试程序结合 libxcam 的 Stitcher/GeoMapper 等组件，
 * 演示从输入文件读入多路鱼眼图像并完成拼接、顶视图、Cubemap 等处理的完整流程。
 * 代码结构主要分为：流对象管理、GeoMapper/Blender 初始化、特征匹配与性能统计、
 * 以及命令行参数解析和主处理循环。注释中将针对各模块的职责、调用路径与关键参数进行说明。
 */

#define ENABLE_FISHEYE_IMG_ROI 0

// 帧模式：Single 代表只读取一帧并循环处理；Multi 则连续读取多帧，适合视频输入。
enum FrameMode {
    FrameSingle = 0,
    FrameMulti
};

// 拼接后端模块枚举：可在命令行选择 CPU Soft、OpenGL ES、Vulkan 等实现。
enum SVModule {
    SVModuleNone    = 0,
    SVModuleSoft,
    SVModuleGLES,
    SVModuleVulkan
};

// 输出配置，决定是否保存拼接结果、顶视图、立方体图及它们对应的输出流索引。
struct SVOutConfig {
    bool save_output = true;
    uint32_t stitch_index = 0;

    bool save_topview = false;
    uint32_t topview_index;

    bool save_cubemap = false;
    uint32_t cubemap_index;

    bool is_save() const {
        return save_output || save_topview || save_cubemap;
    }
};

#if HAVE_GLES

static void dump_dma_video_buf (SmartPtr<VideoBuffer> buf, const char *prefix_name, uint32_t idx)
{
    char file_name[256];
    XCAM_ASSERT (prefix_name);
    XCAM_ASSERT (buf.ptr ());

    const VideoBufferInfo &info = buf->get_video_info ();
    snprintf (
        file_name, 256, "%s-%dx%d.%05d.%s.yuv",
        prefix_name, info.width, info.height, idx, xcam_fourcc_to_string (info.format));

    SmartPtr<GLTexture> tex = GLTexture::create_texture (buf);

    tex->dump_texture_image (file_name);
}

static SmartPtr<DmaVideoBuffer>
convert_to_dma_buffer (SmartPtr<VideoBuffer>& in_buf)
{
    const VideoBufferInfo &in_info = in_buf->get_video_info ();

    uint8_t* buf_data = in_buf->map ();

    SmartPtr<GLTexture> tex = GLTexture::create_texture (buf_data, in_info.width, in_info.height, in_info.format);
    in_buf->unmap ();
    XCAM_FAIL_RETURN (
        ERROR, tex.ptr () != NULL, NULL,
        "gl-dmabuf create texture from buffer failed");

    SmartPtr<DmaVideoBuffer> dma_buf = EGLBase::instance ()->export_dma_buffer (tex).dynamic_cast_ptr<DmaVideoBuffer>();
    XCAM_FAIL_RETURN (
        ERROR, dma_buf.ptr () != NULL, NULL,
        "gl-dmabuf export dma buffer failed");

    const VideoBufferInfo &info = dma_buf->get_video_info ();
    XCAM_LOG_DEBUG ("DMA fd:%d", dma_buf->get_fd ());
    XCAM_LOG_DEBUG ("DmaVideoBuffer width:%d, height:%d, stride:%d, offset:%ld, format:%s", info.width, info.height, info.strides[0], info.offsets[0], xcam_fourcc_to_string (info.format));
    XCAM_LOG_DEBUG ("DmaVideoBuffer: modifiers:%lu, dmabuf fd:%d, fourcc:%s", info.modifiers[0], dma_buf->get_fd (), xcam_fourcc_to_string(info.fourcc));

#ifdef DUMP_TEXTURE
    static uint32_t idx = 0;
    char file_name[256];
    snprintf (file_name, 256, "%s-%dx%d.%05d.%s.yuv",
              "dump_texture", info.width, info.height, idx, xcam_fourcc_to_string (info.format));

    tex->dump_texture_image (file_name);
    dump_dma_video_buf (dma_buf, "dump_dmabuf", idx++);
#endif
    return dma_buf;
}

#endif

/*
 * SVStream 封装单路输入或输出流的基本属性，包括文件名称、分辨率、以及在不同后端下
 * 对应的缓冲池创建逻辑。测试代码通过它来统一管理文件读写、GeoMapper 绑定以及
 * Vulkan 设备句柄等信息，避免在主流程里散落大量条件编译。
 */

class SVStream
    : public Stream
{
public:
    explicit SVStream (const char *file_name = NULL, uint32_t width = 0, uint32_t height = 0);
    virtual ~SVStream () {}

    void set_module (SVModule module) {
        XCAM_ASSERT (module != SVModuleNone);
        _module = module;
    }

    void set_mapper (const SmartPtr<GeoMapper> &mapper) {
        XCAM_ASSERT (mapper.ptr ());
        _mapper = mapper;
    }
    const SmartPtr<GeoMapper> &get_mapper () {
        return _mapper;
    }

#if HAVE_VULKAN
    void set_vk_device (SmartPtr<VKDevice> &device) {
        XCAM_ASSERT (device.ptr ());
        _vk_dev = device;
    }
    SmartPtr<VKDevice> &get_vk_device () {
        return _vk_dev;
    }
#endif

    virtual XCamReturn create_buf_pool (uint32_t reserve_count, uint32_t format = V4L2_PIX_FMT_NV12);

private:
    XCAM_DEAD_COPY (SVStream);

private:
    SVModule               _module;
    SmartPtr<GeoMapper>    _mapper;
#if HAVE_VULKAN
    SmartPtr<VKDevice>     _vk_dev;
#endif
};
typedef std::vector<SmartPtr<SVStream>> SVStreams;

SVStream::SVStream (const char *file_name, uint32_t width, uint32_t height)
    :  Stream (file_name, width, height)
    , _module (SVModuleNone)
{
}

XCamReturn
SVStream::create_buf_pool (uint32_t reserve_count, uint32_t format)
{
    XCAM_ASSERT (get_width () && get_height ());
    XCAM_FAIL_RETURN (
        ERROR, _module != SVModuleNone, XCAM_RETURN_ERROR_PARAM,
        "invalid module, please set module first");

    VideoBufferInfo info;
    info.init (format, get_width (), get_height ());

    SmartPtr<BufferPool> pool;
    if (_module == SVModuleSoft) {
        pool = new SoftVideoBufAllocator (info);
    } else if (_module == SVModuleGLES) {
#if HAVE_GLES
        pool = new GLVideoBufferPool (info);
#endif
    } else if (_module == SVModuleVulkan) {
#if HAVE_VULKAN
        XCAM_ASSERT (_vk_dev.ptr ());
        pool = create_vk_buffer_pool (_vk_dev);
        XCAM_ASSERT (pool.ptr ());
        pool->set_video_info (info);
#endif
    }
    XCAM_ASSERT (pool.ptr ());

    if (!pool->reserve (reserve_count)) {
        XCAM_LOG_ERROR ("create buffer pool failed");
        return XCAM_RETURN_ERROR_MEM;
    }

    set_buf_pool (pool);
    return XCAM_RETURN_NO_ERROR;
}

// 根据所选模块实例化对应的 Stitcher，Soft/GLES/Vulkan 之间共享统一接口。
static SmartPtr<Stitcher>
create_stitcher (const SmartPtr<SVStream> &stitch, SVModule module)
{
    SmartPtr<Stitcher> stitcher;

    if (module == SVModuleSoft) {
        stitcher = Stitcher::create_soft_stitcher ();
    } else if (module == SVModuleGLES) {
#if HAVE_GLES
        stitcher = Stitcher::create_gl_stitcher ();
#endif
    } else if (module == SVModuleVulkan) {
#if HAVE_VULKAN
        SmartPtr<VKDevice> dev = stitch->get_vk_device ();
        XCAM_ASSERT (dev.ptr ());
        stitcher = Stitcher::create_vk_stitcher (dev);
#else
        XCAM_UNUSED (stitch);
#endif
    }
    XCAM_ASSERT (stitcher.ptr ());

    return stitcher;
}

// 拼接输出、顶视图等文件名需要带上语义前缀，此函数负责拼接目录和前缀。
static void
combine_name (const char *orig_name, const char *embedded_str, char *new_name)
{
    const char *dir_delimiter = strrchr (orig_name, '/');

    if (dir_delimiter) {
        std::string path (orig_name, dir_delimiter - orig_name + 1);
        XCAM_ASSERT (path.c_str ());
        snprintf (new_name, XCAM_TEST_MAX_STR_SIZE, "%s%s_%s", path.c_str (), embedded_str, dir_delimiter + 1);
    } else {
        snprintf (new_name, XCAM_TEST_MAX_STR_SIZE, "%s_%s", embedded_str, orig_name);
    }
}

// 根据已有流的文件名自动生成同目录下的新流，并使用相同的分辨率信息。
static void
add_stream (SVStreams &streams, const char *stream_name, uint32_t width, uint32_t height)
{
    char file_name[XCAM_TEST_MAX_STR_SIZE] = {'\0'};
    combine_name (streams[0]->get_file_name (), stream_name, file_name);

    SmartPtr<SVStream> stream = new SVStream (file_name, width, height);
    XCAM_ASSERT (stream.ptr ());
    streams.push_back (stream);
}

// 可选的输入调试输出：在启用 OpenCV 调试时，将原始鱼眼图保存到磁盘以便检查标定质量。
static void
write_in_image (const SmartPtr<Stitcher> &stitcher, const SVStreams &ins, uint32_t frame_num)
{
#if (XCAM_TEST_STREAM_DEBUG) && (XCAM_TEST_OPENCV)
    char img_name[XCAM_TEST_MAX_STR_SIZE] = {'\0'};
    char idx_str[XCAM_TEST_MAX_STR_SIZE] = {'\0'};
    char frame_str[XCAM_TEST_MAX_STR_SIZE] = {'\0'};
    std::snprintf (frame_str, XCAM_TEST_MAX_STR_SIZE, "frame:%d", frame_num);

    if (stitcher->get_dewarp_mode () == DewarpBowl) {
        for (uint32_t i = 0; i < ins.size (); ++i) {
            std::snprintf (idx_str, XCAM_TEST_MAX_STR_SIZE, "idx:%d", i);
            std::snprintf (img_name, XCAM_TEST_MAX_STR_SIZE, "%s//orig_fisheye_%d_%d.jpg", XCAM_TEST_STREAM_FOLDER, frame_num, i);
            ins[i]->debug_write_image (img_name, frame_str, idx_str);
        }
    } else {
        cv::Mat mat;
        const StitchInfo &stitch_info = stitcher->get_stitch_info ();

        if (ins.size () == 1) {
            convert_to_mat (ins[0]->get_buf (), mat);

            for (uint32_t i = 0; i < stitcher->get_camera_num (); i++) {
                const FisheyeInfo &info = stitch_info.fisheye_info[i];
                cv::circle (mat, cv::Point (info.intrinsic.cx, info.intrinsic.cy), info.radius, cv::Scalar(0, 0, 255), 2);
            }
            cv::putText (mat, frame_str, cv::Point(20, 50), cv::FONT_HERSHEY_COMPLEX, 2.0,
                         cv::Scalar(0, 0, 255), 2, 8, false);

            std::snprintf (img_name, XCAM_TEST_MAX_STR_SIZE, "%s//orig_fisheye_%d.jpg", XCAM_TEST_STREAM_FOLDER, frame_num);
            cv::imwrite (img_name, mat);
        } else {
            char idx_str[XCAM_TEST_MAX_STR_SIZE] = {'\0'};
            for (uint32_t i = 0; i < ins.size (); i++) {
                convert_to_mat (ins[i]->get_buf (), mat);

                const FisheyeInfo &info = stitch_info.fisheye_info[i];
                cv::circle (mat, cv::Point (info.intrinsic.cx, info.intrinsic.cy), info.radius, cv::Scalar(0, 0, 255), 2);
                cv::putText (mat, frame_str, cv::Point(20, 50), cv::FONT_HERSHEY_COMPLEX, 2.0,
                             cv::Scalar(0, 0, 255), 2, 8, false);

                std::snprintf (idx_str, XCAM_TEST_MAX_STR_SIZE, "idx:%d", i);
                cv::putText (mat, idx_str, cv::Point (20, 110), cv::FONT_HERSHEY_COMPLEX, 2.0,
                             cv::Scalar (0, 0, 255), 2, 8, false);

                std::snprintf (img_name, XCAM_TEST_MAX_STR_SIZE, "%s//orig_fisheye_%d_%d.jpg", XCAM_TEST_STREAM_FOLDER, frame_num, i);
                cv::imwrite (img_name, mat);
            }
        }
    }
#else
    XCAM_UNUSED (stitcher);
    XCAM_UNUSED (ins);
    XCAM_UNUSED (frame_num);
#endif
}

// 输出调试函数：默认写入原始缓冲，若打开调试宏则同时导出带帧号的图像文件。
static void
write_out_image (const SmartPtr<SVStream> &out, uint32_t frame_num)
{
#if !XCAM_TEST_STREAM_DEBUG
    XCAM_UNUSED (frame_num);
    out->write_buf ();
#else
    char frame_str[XCAM_TEST_MAX_STR_SIZE] = {'\0'};
    std::snprintf (frame_str, XCAM_TEST_MAX_STR_SIZE, "frame:%d", frame_num);
    out->write_buf (frame_str);

#if XCAM_TEST_OPENCV
    char img_name[XCAM_TEST_MAX_STR_SIZE] = {'\0'};
    std::snprintf (img_name, XCAM_TEST_MAX_STR_SIZE, "%s_%d.jpg", out->get_file_name (), frame_num);
    out->debug_write_image (img_name, frame_str);
#endif
#endif
}

// 依据 Bowl 模型生成顶视图重映射表，并为目标流绑定 GeoMapper。
static XCamReturn
create_topview_mapper (
    const SmartPtr<Stitcher> &stitcher, const SmartPtr<SVStream> &stitch,
    const SmartPtr<SVStream> &topview, SVModule module)
{
    BowlModel bowl_model (stitcher->get_bowl_config (), stitch->get_width (), stitch->get_height ());
    BowlModel::PointMap points;

    float length_mm = 0.0f, width_mm = 0.0f;
    bowl_model.get_max_topview_area_mm (length_mm, width_mm);
    XCAM_LOG_INFO ("Max Topview Area (L%.2fmm, W%.2fmm)", length_mm, width_mm);

    bowl_model.get_topview_rect_map (points, topview->get_width (), topview->get_height (), length_mm, width_mm);
    SmartPtr<GeoMapper> mapper;
    if (module == SVModuleSoft) {
        mapper = GeoMapper::create_soft_geo_mapper ();
    } else if (module == SVModuleGLES) {
#if HAVE_GLES
        mapper = GeoMapper::create_gl_geo_mapper ();
#endif
    } else if (module == SVModuleVulkan) {
#if HAVE_VULKAN
        SmartPtr<VKDevice> dev = stitch->get_vk_device ();
        XCAM_ASSERT (dev.ptr ());
        mapper = GeoMapper::create_vk_geo_mapper (dev, "topview-map");
#endif
    }
    XCAM_ASSERT (mapper.ptr ());

    mapper->set_output_size (topview->get_width (), topview->get_height ());
    mapper->set_lookup_table (points.data (), topview->get_width (), topview->get_height ());
    /* TODO: save the mapper */
    topview->set_mapper (mapper);

    return XCAM_RETURN_NO_ERROR;
}

// 通用的重映射入口：利用前面创建的查找表把拼接图转换为其它视角。
static XCamReturn
remap_buf (const SmartPtr<SVStream> &stitch, const SmartPtr<SVStream> &topview)
{
    const SmartPtr<GeoMapper> mapper = topview->get_mapper();
    XCAM_ASSERT (mapper.ptr ());

    XCamReturn ret = mapper->remap (stitch->get_buf (), topview->get_buf ());
    if (ret != XCAM_RETURN_NO_ERROR) {
        XCAM_LOG_ERROR ("remap stitched image to topview failed.");
        return ret;
    }

    return XCAM_RETURN_NO_ERROR;
}

// 构建 Cubemap 重映射器，核心区别在于使用 CubeMapModel 生成六面展开 LUT。
static XCamReturn
create_cubemap_mapper (
    const SmartPtr<Stitcher> &stitcher, const SmartPtr<SVStream> &stitch,
    const SmartPtr<SVStream> &cubemap, SVModule module)
{
    CubeMapModel cubemap_model (stitch->get_width (), stitch->get_height ());
    CubeMapModel::PointMap points;

    cubemap_model.get_cubemap_rect_map (points, cubemap->get_width (), cubemap->get_height ());
    SmartPtr<GeoMapper> mapper;
    if (module == SVModuleSoft) {
        mapper = GeoMapper::create_soft_geo_mapper ();
    } else if (module == SVModuleGLES) {
#if HAVE_GLES
        mapper = GeoMapper::create_gl_geo_mapper ();
#endif
    } else if (module == SVModuleVulkan) {
#if HAVE_VULKAN
        SmartPtr<VKDevice> dev = stitch->get_vk_device ();
        XCAM_ASSERT (dev.ptr ());
        mapper = GeoMapper::create_vk_geo_mapper (dev, "cubemap-map");
#endif
    }
    XCAM_ASSERT (mapper.ptr ());

    mapper->set_output_size (cubemap->get_width (), cubemap->get_height ());
    mapper->set_lookup_table (points.data (), cubemap->get_width (), cubemap->get_height ());
    cubemap->set_mapper (mapper);

    return XCAM_RETURN_NO_ERROR;
}

// 写出一帧所有需要的结果：包含主拼接、可选的顶视图和立方体输出。
static void
write_image (
    const SmartPtr<Stitcher> &stitcher,
    const SVStreams &ins, const SVStreams &outs,
    const SVOutConfig &out_config)
{
    static uint32_t frame_num = 0;

    write_in_image (stitcher, ins, frame_num);

    if (out_config.save_output) {
        // Bowl view 2D
        write_out_image (outs[out_config.stitch_index], frame_num);
    }

    if (out_config.save_topview) {
        // Topview 2D
        remap_buf (outs[out_config.stitch_index], outs[out_config.topview_index]);
        write_out_image (outs[out_config.topview_index], frame_num);
    }

    if (out_config.save_cubemap) {
        remap_buf (outs[out_config.stitch_index], outs[out_config.cubemap_index]);
        write_out_image (outs[out_config.cubemap_index], frame_num);
    }

    frame_num++;
}

// 判断特征匹配是否已经稳定：用于控制 FPS 统计和输出时机。
static bool
stable_stitch (const SmartPtr<Stitcher> &stitcher)
{
    return (
               stitcher->get_fm_mode () == FMNone ||
               stitcher->get_fm_status () == FMStatusWholeWay ||
               stitcher->get_fm_frame_count () > stitcher->get_fm_frames ());
}

XCAM_OBJ_PROFILING_DEFINES;

// 单帧模式：预先读取所有输入，再多次调用 stitch_buffers，可用于压力测试。
static int
single_frame (
    const SmartPtr<Stitcher> &stitcher,
    const SVStreams &ins, const SVStreams &outs,
    const SVOutConfig &out_config, int loop, bool enable_dmabuf = false)
{
    for (uint32_t i = 0; i < ins.size (); ++i) {
        CHECK (ins[i]->rewind (), "rewind buffer from file(%s) failed", ins[i]->get_file_name ());
    }

    VideoBufferList in_buffers;
    for (uint32_t i = 0; i < ins.size (); ++i) {
        XCamReturn ret = ins[i]->read_buf ();
        CHECK_EXP (ret == XCAM_RETURN_NO_ERROR, "read buffer from file(%s) failed.", ins[i]->get_file_name ());

        XCAM_ASSERT (ins[i]->get_buf ().ptr ());

        // GLES 路径下可选择把 CPU 缓冲转换成 dmabuf，以验证零拷贝流程。
        if (enable_dmabuf) {
#if HAVE_GLES
            SmartPtr<DmaVideoBuffer> dma_buf = convert_to_dma_buffer (ins[i]->get_buf ());
            in_buffers.push_back (dma_buf);
#else
            XCAM_LOG_ERROR ("GLES module is unsupported");
#endif
        } else {
            in_buffers.push_back (ins[i]->get_buf ());
        }
    }

    while (loop--) {
        XCAM_OBJ_PROFILING_START;

        SmartPtr<VideoBuffer> out_dma_buf;
        if (enable_dmabuf) {
#if HAVE_GLES
            // 输出同样使用 dmabuf，可直接交给 GLES 管线进一步处理。
            out_dma_buf = convert_to_dma_buffer (outs[out_config.stitch_index]->get_buf ());
            CHECK (stitcher->stitch_buffers (in_buffers, out_dma_buf), "stitch buffer failed.");
#else
            XCAM_LOG_ERROR ("GLES module is unsupported");
#endif
        } else {
            CHECK (stitcher->stitch_buffers (in_buffers, outs[out_config.stitch_index]->get_buf ()), "stitch buffer failed.");
        }

        XCAM_OBJ_PROFILING_END ("stitch-buffers", XCAM_OBJ_DUR_FRAME_NUM);

        if (out_config.is_save()) {
            if (stitcher->complete_stitch ()) {
                if (enable_dmabuf) {
#if HAVE_GLES
                    dump_dma_video_buf (out_dma_buf, "test-surround-view-output-dma-buffer", loop);
#else
                    XCAM_LOG_ERROR ("GLES module is unsupported");
#endif
                } else {
                    write_image (stitcher, ins, outs, out_config);
                }
            }
        }

        if (stable_stitch (stitcher)) {
            FPS_CALCULATION (surround_view, XCAM_OBJ_DUR_FRAME_NUM);
        }
    }

    return 0;
}

// 多帧模式：循环从文件读取直到结束，适合处理视频序列或多帧原始数据。
static int
multi_frame (
    const SmartPtr<Stitcher> &stitcher,
    const SVStreams &ins, const SVStreams &outs,
    const SVOutConfig &out_config, int loop)
{
    XCamReturn ret = XCAM_RETURN_NO_ERROR;

    VideoBufferList in_buffers;                                       // 用于暂存当前帧的所有输入缓冲
    while (loop--) {                                                  // 支持外层多次循环播放输入序列
        for (uint32_t i = 0; i < ins.size (); ++i) {
            // 多帧模式需要反复从文件读取，因此每次进入 loop 时都将文件指针重置到开头。
            CHECK (ins[i]->rewind (), "rewind buffer from file(%s) failed", ins[i]->get_file_name ());
        }

        do {
            in_buffers.clear ();                                      // 一次拼接前清空输入列表

            for (uint32_t i = 0; i < ins.size (); ++i) {
                ret = ins[i]->read_buf ();                            // 逐路拉取下一帧数据
                if (ret == XCAM_RETURN_BYPASS)                        // 若某路到达文件末尾，则结束本轮
                    break;
                CHECK (ret, "read buffer from file(%s) failed.", ins[i]->get_file_name ());

                in_buffers.push_back (ins[i]->get_buf ());            // 收集读到的缓冲供 Stitcher 使用
            }
            if (ret == XCAM_RETURN_BYPASS)
                break;

            XCAM_OBJ_PROFILING_START;                                 // 性能统计起始

            CHECK (
                stitcher->stitch_buffers (in_buffers, outs[out_config.stitch_index]->get_buf ()),
                "stitch buffer failed.");                             // 将当前帧输入传给 Stitcher 完成拼接

            XCAM_OBJ_PROFILING_END ("stitch-buffers", XCAM_OBJ_DUR_FRAME_NUM);

            // 若配置需要保存结果，并且 Stitcher 的特征匹配流程已经达到稳定状态，则写出文件
            if (out_config.is_save()) {
                if (stitcher->complete_stitch ()) {
                    write_image (stitcher, ins, outs, out_config);
                }
            }

            // 拼接稳定后统计 FPS，避免特征匹配预热阶段影响平均值
            if (stable_stitch (stitcher)) {
                FPS_CALCULATION (surround_view, XCAM_OBJ_DUR_FRAME_NUM);
            }
        } while (true);                                               // 持续读取直到某路触发 BYPASS
    }

    return 0;
}

// 根据帧模式选择不同运行路径，同时初始化性能统计与输入流检查。
static int
run_stitcher (
    const SmartPtr<Stitcher> &stitcher,
    const SVStreams &ins, const SVStreams &outs,
    FrameMode frame_mode, const SVOutConfig &out_config, int loop, bool enable_dmabuf = false)
{
    XCAM_OBJ_PROFILING_INIT;

    CHECK (check_streams<SVStreams> (ins), "invalid input streams");
    CHECK (check_streams<SVStreams> (outs), "invalid output streams");

    int ret = -1;
    if (frame_mode == FrameSingle)
        ret = single_frame (stitcher, ins, outs, out_config, loop, enable_dmabuf);
    else if (frame_mode == FrameMulti)
        ret = multi_frame (stitcher, ins, outs, out_config, loop);
    else
        XCAM_LOG_ERROR ("invalid frame mode: %d", frame_mode);

    return ret;
}

// 打印命令行帮助信息，列出所有可配置的拼接参数。
static void usage(const char* arg0)
{
    printf ("Usage:\n"
            "%s --module MODULE --input input0.nv12 --input input1.nv12 --input input2.nv12 ...\n"
            "\t--module            processing module, selected from: soft, gles, vulkan\n"
            "\t--dma               enable input/output dmabuf\n"
            "\t                    read calibration files from exported path $FISHEYE_CONFIG_PATH\n"
            "\t--input             input image(NV12)\n"
            "\t--output            output image(NV12/MP4)\n"
            "\t--in-w              optional, input width, default: 1280\n"
            "\t--in-h              optional, input height, default: 800\n"
            "\t--out-w             optional, output width, default: 1920\n"
            "\t--out-h             optional, output height, default: 640\n"
            "\t--topview-w         optional, output width, default: 1280\n"
            "\t--topview-h         optional, output height, default: 720\n"
            "\t--in-format         optional, pixel format, select from [nv12/yuv], default: nv12\n"
            "\t--fisheye-num       optional, the number of fisheye lens, default: 4\n"
            "\t--cam-model         optional, camera model\n"
            "\t                    select from [cama2c1080p/camb4c1080p/camc3c4k/camc3c8k/camc6c8k/camd3c8k/camd6c8k], default: camb4c1080p\n"
            "\t--blend-pyr-levels  optional, the pyramid levels of blender, default: 2\n"
            "\t--dewarp-mode       optional, fisheye dewarp mode, select from [sphere/bowl], default: bowl\n"
            "\t--scopic-mode       optional, scopic mode, select from [mono/stereoleft/stereoright], default: mono\n"
            "\t--scale-mode        optional, scaling mode for geometric mapping,\n"
            "\t                    select from [singleconst/dualconst/dualcurve], default: singleconst\n"
#if HAVE_OPENCV
            "\t--fm-mode           optional, feature match mode,\n"
            "\t                    select from [none/default/cluster/capi], default: none\n"
            "\t--fm-frames         optional, how many frames need to run feature match at the beginning, default: 100\n"
            "\t--fm-status         optional, running status of feature match,\n"
            "\t                    select from [wholeway/halfway/fmfirst], default: wholeway\n"
            "\t                    wholeway: run feature match during the entire runtime\n"
            "\t                    halfway: run feature match with stitching in the first --fm-frames frames\n"
            "\t                    fmfirst: run feature match without stitching in the first --fm-frames frames\n"
#else
            "\t--fm-mode           optional, feature match mode, select from [none], default: none\n"
#endif
            "\t--frame-mode        optional, times of buffer reading, select from [single/multi], default: multi\n"
            "\t--save              optional, save file or not, select from [true/false], default: true\n"
            "\t--save-topview      optional, save top view video, select from [true/false], default: false\n"
            "\t--save-cubemap      optional, save cubemap video, select from [true/false], default: false\n"
            "\t--loop              optional, how many loops need to run, default: 1\n"
            "\t--help              usage\n",
            arg0);
}

int main (int argc, char *argv[])
{
    /* 单个鱼眼相机的默认采集分辨率，可通过 --in-w/--in-h 覆盖 */
    uint32_t input_width = 1280;
    uint32_t input_height = 800;
    /*
     * 主拼接结果（环视 ERP/Bowl）的输出分辨率。与 Stitcher::estimate_round_slices 中的计算配合，
     * 输出宽度会决定每个回转切片的像素宽度，并且根据 `_alignment_x` 对齐到 8/16 像素，
     * 这会导致真实的切片角度与原始 angle_range 之间存在极小偏差（通常允许）。
     */
    uint32_t output_width = 1920;
    uint32_t output_height = 640;
    /* 如果启用 --save-topview true，生成顶视图时使用的输出大小 */
    uint32_t topview_width = 1280;
    uint32_t topview_height = 720;
    /* 启用 --save-cubemap true 时的立方体贴图尺寸 */
    uint32_t cubemap_width = 1280;
    uint32_t cubemap_height = 720;

    SVStreams ins;   // 输入流容器，支持单路或多路鱼眼文件
    SVStreams outs;  // 输出流容器，第一个元素始终为主拼接结果

    /* 输入像素格式，默认 NV12，可切换到 YUV420 */
    uint32_t input_format = V4L2_PIX_FMT_NV12;

    uint32_t fisheye_num = 4;
    CamModel cam_model = CamB4C1080P;
    FrameMode frame_mode = FrameMulti;
    SVModule module = SVModuleNone;
    const char* device_node = NULL;
    GeoMapScaleMode scale_mode = ScaleSingleConst;
    FeatureMatchMode fm_mode = FMNone;
    FisheyeDewarpMode dewarp_mode = DewarpBowl;
    StitchScopicMode scopic_mode = ScopicMono;

    uint32_t blend_pyr_levels = 2;

    bool enable_dmabuf = false;

#if HAVE_OPENCV
    uint32_t fm_frames = 100;
    FeatureMatchStatus fm_status = FMStatusWholeWay;
#endif

    int loop = 1;
    int repeat = 1;
    SVOutConfig out_config;  // 控制是否输出拼接/顶视图/Cubemap

    /* getopt_long 参数描述表：列出所有命令行开关与其缩写 */
    const struct option long_opts[] = {
        {"module", required_argument, NULL, 'm'},
        {"dma", required_argument, NULL, 'M'},
        {"device-node", required_argument, NULL, 'D'},
        {"input", required_argument, NULL, 'i'},
        {"output", required_argument, NULL, 'o'},
        {"in-w", required_argument, NULL, 'w'},
        {"in-h", required_argument, NULL, 'h'},
        {"out-w", required_argument, NULL, 'W'},
        {"out-h", required_argument, NULL, 'H'},
        {"topview-w", required_argument, NULL, 'P'},
        {"topview-h", required_argument, NULL, 'V'},
        {"cubemap-w", required_argument, NULL, 'X'},
        {"cubemap-h", required_argument, NULL, 'Y'},
        {"in-format", required_argument, NULL, 'p'},
        {"fisheye-num", required_argument, NULL, 'N'},
        {"cam-model", required_argument, NULL, 'C'},
        {"blend-pyr-levels", required_argument, NULL, 'b'},
        {"dewarp-mode", required_argument, NULL, 'd'},
        {"scopic-mode", required_argument, NULL, 'c'},
        {"scale-mode", required_argument, NULL, 'S'},
        {"fm-mode", required_argument, NULL, 'F'},
#if HAVE_OPENCV
        {"fm-frames", required_argument, NULL, 'n'},
        {"fm-status", required_argument, NULL, 'T'},
#endif
        {"frame-mode", required_argument, NULL, 'f'},
        {"save", required_argument, NULL, 's'},
        {"save-topview", required_argument, NULL, 't'},
        {"save-cubemap", required_argument, NULL, 'q'},
        {"loop", required_argument, NULL, 'L'},
        {"repeat", required_argument, NULL, 'R'},
        {"help", no_argument, NULL, 'e'},
        {NULL, 0, NULL, 0},
    };

    int opt = -1;
    /* 主循环解析命令行参数，将字符串转换成内部配置 */
    while ((opt = getopt_long(argc, argv, "", long_opts, NULL)) != -1) {
        switch (opt) {
        case 'm':
            XCAM_ASSERT (optarg);
            if (!strcasecmp (optarg, "soft"))
                module = SVModuleSoft;
            else if (!strcasecmp (optarg, "gles")) {
                module = SVModuleGLES;
            } else if (!strcasecmp (optarg, "vulkan")) {
                module = SVModuleVulkan;
            } else {
                XCAM_LOG_ERROR ("unknown module: %s", optarg);
                usage (argv[0]);
                return -1;
            }
            break;
        case 'M':
            enable_dmabuf = (strcasecmp (optarg, "false") == 0 ? false : true);
            break;
        case 'D':
            XCAM_ASSERT (optarg);
            device_node = optarg;
            break;

        case 'i':
            XCAM_ASSERT (optarg);
            PUSH_STREAM (SVStream, ins, optarg);
            break;
        case 'o':
            XCAM_ASSERT (optarg);
            PUSH_STREAM (SVStream, outs, optarg);
            break;
        case 'w':
            input_width = atoi(optarg);
            break;
        case 'h':
            input_height = atoi(optarg);
            break;
        case 'W':
            output_width = atoi(optarg);
            break;
        case 'H':
            output_height = atoi(optarg);
            break;
        case 'p':
            if (!strcasecmp (optarg, "nv12"))
                input_format = V4L2_PIX_FMT_NV12;
            else if (!strcasecmp (optarg, "yuv"))
                input_format = V4L2_PIX_FMT_YUV420;
            else {
                XCAM_LOG_ERROR ("unsupported input format: %s", optarg);
                usage (argv[0]);
                return -1;
            }
            break;
        case 'P':
            topview_width = atoi(optarg);
            break;
        case 'V':
            topview_height = atoi(optarg);
            break;
        case 'X':
            cubemap_width = atoi(optarg);
            break;
        case 'Y':
            cubemap_height = atoi(optarg);
            break;
        case 'N':
            fisheye_num = atoi(optarg);
            if (fisheye_num > XCAM_STITCH_FISHEYE_MAX_NUM) {
                XCAM_LOG_ERROR ("fisheye number should not be greater than %d\n", XCAM_STITCH_FISHEYE_MAX_NUM);
                return -1;
            }
            break;
        case 'C':
            if (!strcasecmp (optarg, "cama2c1080p"))
                cam_model = CamA2C1080P;
            else if (!strcasecmp (optarg, "camb4c1080p"))
                cam_model = CamB4C1080P;
            else if (!strcasecmp(optarg, "camc3c4k"))
                cam_model = CamC3C4K;
            else if (!strcasecmp (optarg, "camc3c8k"))
                cam_model = CamC3C8K;
            else if (!strcasecmp (optarg, "camc6c8k"))
                cam_model = CamC6C8K;
            else if (!strcasecmp (optarg, "camd3c8k"))
                cam_model = CamD3C8K;
            else if (!strcasecmp (optarg, "camd6c8k"))
                cam_model = CamD6C8K;
            else {
                XCAM_LOG_ERROR ("incorrect camera model: %s", optarg);
                usage (argv[0]);
                return -1;
            }
            break;
        case 'b':
            blend_pyr_levels = atoi(optarg);
            break;
        case 'd':
            if (!strcasecmp (optarg, "sphere"))
                dewarp_mode = DewarpSphere;
            else if(!strcasecmp (optarg, "bowl"))
                dewarp_mode = DewarpBowl;
            else {
                XCAM_LOG_ERROR ("incorrect fisheye dewarp mode: %s", optarg);
                usage (argv[0]);
                return -1;
            }
            break;
        case 'c':
            if (!strcasecmp (optarg, "mono"))
                scopic_mode = ScopicMono;
            else if(!strcasecmp (optarg, "stereoleft"))
                scopic_mode = ScopicStereoLeft;
            else if(!strcasecmp (optarg, "stereoright"))
                scopic_mode = ScopicStereoRight;
            else {
                XCAM_LOG_ERROR ("incorrect scopic mode: %s", optarg);
                usage (argv[0]);
                return -1;
            }
            break;
        case 'S':
            XCAM_ASSERT (optarg);
            if (!strcasecmp (optarg, "singleconst"))
                scale_mode = ScaleSingleConst;
            else if (!strcasecmp (optarg, "dualconst"))
                scale_mode = ScaleDualConst;
            else if (!strcasecmp (optarg, "dualcurve"))
                scale_mode = ScaleDualCurve;
            else {
                XCAM_LOG_ERROR ("GeoMapScaleMode unknown mode: %s", optarg);
                usage (argv[0]);
                return -1;
            }
            break;
        case 'F':
            XCAM_ASSERT (optarg);
            if (!strcasecmp (optarg, "none"))
                fm_mode = FMNone;
#if HAVE_OPENCV
            else if (!strcasecmp (optarg, "default"))
                fm_mode = FMDefault;
            else if (!strcasecmp (optarg, "cluster"))
                fm_mode = FMCluster;
            else if (!strcasecmp (optarg, "capi"))
                fm_mode = FMCapi;
#endif
            else {
                XCAM_LOG_ERROR ("surround view unsupported feature match mode: %s", optarg);
                usage (argv[0]);
                return -1;
            }
            break;
#if HAVE_OPENCV
        case 'n':
            fm_frames = atoi(optarg);
            break;
        case 'T':
            XCAM_ASSERT (optarg);
            if (!strcasecmp (optarg, "wholeway"))
                fm_status = FMStatusWholeWay;
            else if (!strcasecmp (optarg, "halfway"))
                fm_status = FMStatusHalfWay;
            else if (!strcasecmp (optarg, "fmfirst"))
                fm_status = FMStatusFMFirst;
            else {
                XCAM_LOG_ERROR ("surround view unsupported feature match status: %s", optarg);
                usage (argv[0]);
                return -1;
            }
            break;
#endif
        case 'f':
            XCAM_ASSERT (optarg);
            if (!strcasecmp (optarg, "single"))
                frame_mode = FrameSingle;
            else if (!strcasecmp (optarg, "multi"))
                frame_mode = FrameMulti;
            else {
                XCAM_LOG_ERROR ("FrameMode unknown mode: %s", optarg);
                usage (argv[0]);
                return -1;
            }
            break;
        case 's':
            out_config.save_output = (strcasecmp (optarg, "false") == 0 ? false : true);
            break;
        case 't':
            out_config.save_topview = (strcasecmp (optarg, "false") == 0 ? false : true);
            break;
        case 'q':
            out_config.save_cubemap = (strcasecmp (optarg, "false") == 0 ? false : true);
            break;
        case 'L':
            loop = atoi(optarg);
            break;
        case 'R':
            repeat = atoi(optarg);
            break;
        case 'e':
            usage (argv[0]);
            return 0;
        default:
            XCAM_LOG_ERROR ("getopt_long return unknown value: %c", opt);
            usage (argv[0]);
            return -1;
        }
    }

    if (optind < argc || argc < 2) {
        XCAM_LOG_ERROR ("unknown option %s", argv[optind]);
        usage (argv[0]);
        return -1;
    }

    // 基础合法性检查：输入流数量需与鱼眼数量匹配，输出至少包含拼接主流。
    if ((ins.size () != 1) && (ins.size () != fisheye_num)) {
        XCAM_LOG_ERROR (
            "multiple-input mode: conflicting input number(%lu) and fisheye number(%d)", ins.size (), fisheye_num);
        return -1;
    }

    for (uint32_t i = 0; i < ins.size (); ++i) {
        CHECK_EXP (ins[i].ptr (), "input stream is NULL, index:%d", i);
        CHECK_EXP (strlen (ins[i]->get_file_name ()), "input file name was not set, index:%d", i);
    }

    CHECK_EXP (outs.size () == 1 && outs[out_config.stitch_index].ptr (), "surrond view needs 1 output stream");
    CHECK_EXP (strlen (outs[out_config.stitch_index]->get_file_name ()), "output file name was not set");

    // 输出当前配置，便于在命令行查看最终生效的参数组合。
    for (uint32_t i = 0; i < ins.size (); ++i) {
        printf ("input%d file:\t\t%s\n", i, ins[i]->get_file_name ());
    }
    printf ("camera model:\t\t%s\n", cam_model == CamA2C1080P ? "cama2c1080p" :
            (cam_model == CamB4C1080P ? "camb4c1080p" : (cam_model == CamC3C4K ? "camc3c4k" : (cam_model == CamC3C8K ? "camc3c8k" :
                    (cam_model == CamC6C8K ? "camc6c8k" : (cam_model == CamD3C8K ? "camd3c8k" : "camd6c8k"))))));
    printf ("fisheye number:\t\t%d\n", fisheye_num);
    printf ("stitch module:\t\t%s\n", module == SVModuleGLES ? "GLES" :
            (module == SVModuleVulkan ? "Vulkan" : (module == SVModuleSoft ? "Soft" : "Unknown")));
    printf ("enable DMA buffer input/output:\t\t%s\n", enable_dmabuf ? "true" : "false");
    printf ("device node:\t\t%s\n", device_node != NULL ? device_node : "Not specified, use default model");
    printf ("output file:\t\t%s\n", outs[out_config.stitch_index]->get_file_name ());
    printf ("input width:\t\t%d\n", input_width);
    printf ("input height:\t\t%d\n", input_height);
    printf ("output width:\t\t%d\n", output_width);
    printf ("output height:\t\t%d\n", output_height);
    printf ("topview width:\t\t%d\n", topview_width);
    printf ("topview height:\t\t%d\n", topview_height);
    printf ("cubemap width:\t\t%d\n", cubemap_width);
    printf ("cubemap height:\t\t%d\n", cubemap_height);
    printf ("input format:\t\t%s\n", input_format == V4L2_PIX_FMT_YUV420 ? "yuv" : "nv12");
    printf ("blend pyr levels:\t%d\n", blend_pyr_levels);
    printf ("dewarp mode: \t\t%s\n", dewarp_mode == DewarpSphere ? "sphere" : "bowl");
    printf ("scopic mode:\t\t%s\n", (scopic_mode == ScopicMono) ? "mono" :
            ((scopic_mode == ScopicStereoLeft) ? "stereoleft" : "stereoright"));
    printf ("scaling mode:\t\t%s\n", (scale_mode == ScaleSingleConst) ? "singleconst" :
            ((scale_mode == ScaleDualConst) ? "dualconst" : "dualcurve"));
    printf ("feature match:\t\t%s\n", (fm_mode == FMNone) ? "none" :
            ((fm_mode == FMDefault ) ? "default" : ((fm_mode == FMCluster) ? "cluster" : "capi")));
#if HAVE_OPENCV
    printf ("feature match frames:\t%d\n", fm_frames);
    printf ("feature match status:\t%s\n", (fm_status == FMStatusWholeWay) ? "wholeway" :
            ((fm_status == FMStatusHalfWay) ? "halfway" : "fmfirst"));
#endif
    printf ("frame mode:\t\t%s\n", (frame_mode == FrameSingle) ? "singleframe" : "multiframe");
    printf ("save output:\t\t%s\n", out_config.save_output ? "true" : "false");
    printf ("save topview:\t\t%s\n", out_config.save_topview ? "true" : "false");
    printf ("save cubemap:\t\t%s\n", out_config.save_cubemap ? "true" : "false");
    printf ("loop count:\t\t%d\n", loop);
    printf ("repeat count:\t\t%d\n", repeat);

#if HAVE_GLES
    SmartPtr<EGLBase> egl;
    if (module == SVModuleGLES) {
        if (scale_mode == ScaleDualCurve) {
            XCAM_LOG_ERROR ("GLES module does not support dualcurve scale mode currently");
            return -1;
        }

        egl = EGLBase::instance ();
        XCAM_ASSERT (egl.ptr ());

        // 未指定设备节点时使用默认 render 节点；否则按用户传入的 DRM 设备初始化。
        if (NULL == device_node) {
            XCAM_FAIL_RETURN (ERROR, egl->init (), -1, "init EGL failed");
        } else {
            XCAM_FAIL_RETURN (ERROR, egl->init (device_node), -1, "init EGL failed");
        }
    }
#else
    if (module == SVModuleGLES) {
        XCAM_LOG_ERROR ("GLES module is unsupported");
        return -1;
    }
#endif

    if (module == SVModuleVulkan) {
#if HAVE_VULKAN
        if (scale_mode != ScaleSingleConst) {
            XCAM_LOG_ERROR ("vulkan module only support singleconst scale mode currently");
            return -1;
        }

        // Vulkan 模块共用一个默认设备，并同时绑定到输入/输出流。
        SmartPtr<VKDevice> vk_dev = VKDevice::default_device ();
        for (uint32_t i = 0; i < ins.size (); ++i) {
            ins[i]->set_vk_device (vk_dev);
        }
        XCAM_ASSERT (outs[out_config.stitch_index].ptr ());
        outs[out_config.stitch_index]->set_vk_device (vk_dev);
#else
        XCAM_LOG_ERROR ("vulkan module is unsupported");
        return -1;
#endif
    }

#if ENABLE_FISHEYE_IMG_ROI
    // 针对部分相机型号，可手动裁剪有效鱼眼区域以减少冗余像素的开销。
    if (module == SVModuleGLES && (cam_model == CamC3C4K || cam_model == CamC3C8K || cam_model == CamC6C8K ||
                                   cam_model == CamD3C8K || cam_model == CamD6C8K)) {
        StitchInfo info = stitch_info (cam_model, scopic_mode);

        get_fisheye_info (cam_model, scopic_mode, info.fisheye_info);

        uint32_t *roi_radius = new uint32_t[XCAM_STITCH_FISHEYE_MAX_NUM];
        get_fisheye_img_roi_radius (cam_model, scopic_mode, roi_radius);

        // 针对高分辨率机型，设置 GLES 后端的图像 ROI，避免无效区域参与拼接。
        for (uint32_t i = 0; i < ins.size (); i++) {
            SmartPtr<FisheyeImageFile> file = new FisheyeImageFile ();
            XCAM_ASSERT (file.ptr ());
            file->set_img_size (input_width, input_height);

            FisheyeInfo &fisheye_info = info.fisheye_info[i];
            file->set_center (fisheye_info.intrinsic.cx, fisheye_info.intrinsic.cy);
            file->set_roi_radius (roi_radius[i]);
            ins[i]->set_file (file);
        }
        delete [] roi_radius;
    }
#endif

    // 初始化所有输入流的缓冲池与文件句柄。
    for (uint32_t i = 0; i < ins.size (); ++i) {
        ins[i]->set_module (module);
        ins[i]->set_buf_size (input_width, input_height);
        CHECK (ins[i]->create_buf_pool (6, input_format), "create buffer pool failed");
        CHECK (ins[i]->open_reader ("rb"), "open input file(%s) failed", ins[i]->get_file_name ());
    }

    outs[out_config.stitch_index]->set_buf_size (output_width, output_height);
    if (enable_dmabuf) {
#if HAVE_GLES
        outs[out_config.stitch_index]->set_module (module);
        CHECK (outs[out_config.stitch_index]->create_buf_pool (XCAM_GL_RESERVED_BUF_COUNT, input_format), "create buffer pool failed");
#else
        XCAM_LOG_ERROR ("GLES module is unsupported");
#endif
    }

    if (out_config.save_output) {
        CHECK (outs[out_config.stitch_index]->estimate_file_format (),
               "%s: estimate file format failed", outs[out_config.stitch_index]->get_file_name ());
        CHECK (outs[out_config.stitch_index]->open_writer ("wb"), "open output file(%s) failed", outs[out_config.stitch_index]->get_file_name ());
    }

    while (repeat--) {

        XCAM_LOG_DEBUG ("create stitcher and run test, remain repeat %d times", repeat);

        // 每轮重新创建 Stitcher 以便验证重复运行的稳定性。
        SmartPtr<Stitcher> stitcher = create_stitcher (outs[out_config.stitch_index], module);
        XCAM_ASSERT (stitcher.ptr ());

        stitcher->set_camera_num (fisheye_num);
        stitcher->set_output_size (output_width, output_height);
        stitcher->set_dewarp_mode (dewarp_mode);
        stitcher->set_scale_mode (scale_mode);
        stitcher->set_blend_pyr_levels (blend_pyr_levels);
        stitcher->set_fm_mode (fm_mode);
#if HAVE_OPENCV
        stitcher->set_fm_frames (fm_frames);
        stitcher->set_fm_status (fm_status);
        FMConfig cfg = fm_config (cam_model);
        stitcher->set_fm_config (cfg);
        if (dewarp_mode == DewarpSphere) {
            stitcher->set_fm_region_ratio (fm_region_ratio (cam_model));
        }
#endif

        // 视角范围决定每个相机在环视输出上的切片角度，这里根据车型读取默认配置。
        float *vp_range = new float[XCAM_STITCH_FISHEYE_MAX_NUM];
        stitcher->set_viewpoints_range (viewpoints_range (cam_model, vp_range));
        delete [] vp_range;

        if (dewarp_mode == DewarpSphere) {
            StitchInfo info = stitch_info (cam_model, scopic_mode);

            get_fisheye_info (cam_model, scopic_mode, info.fisheye_info);

            // 打印每路相机的标定参数，方便确认 JSON/文本配置是否加载成功。
            for (uint32_t cam_id = 0; cam_id < XCAM_STITCH_FISHEYE_MAX_NUM; cam_id++) {
                XCAM_LOG_INFO ("cam[%d]: flip=%d ", cam_id, info.fisheye_info[cam_id].intrinsic.flip);
                XCAM_LOG_INFO ("fx=%f ", info.fisheye_info[cam_id].intrinsic.fx);
                XCAM_LOG_INFO ("fy=%f ", info.fisheye_info[cam_id].intrinsic.fy);
                XCAM_LOG_INFO ("cx=%f ", info.fisheye_info[cam_id].intrinsic.cx);
                XCAM_LOG_INFO ("cy=%f ", info.fisheye_info[cam_id].intrinsic.cy);
                XCAM_LOG_INFO ("w=%d ", info.fisheye_info[cam_id].intrinsic.width);
                XCAM_LOG_INFO ("h=%d ", info.fisheye_info[cam_id].intrinsic.height);
                XCAM_LOG_INFO ("fov=%f ", info.fisheye_info[cam_id].intrinsic.fov);
                XCAM_LOG_INFO ("skew=%f ", info.fisheye_info[cam_id].intrinsic.skew);
                XCAM_LOG_INFO ("radius=%f ", info.fisheye_info[cam_id].radius);
                XCAM_LOG_INFO ("distroy coeff=%f %f %f %f ", info.fisheye_info[cam_id].distort_coeff[0], info.fisheye_info[cam_id].distort_coeff[1], info.fisheye_info[cam_id].distort_coeff[2], info.fisheye_info[cam_id].distort_coeff[3]);
                XCAM_LOG_INFO ("fisheye eluer angles: yaw:%f, pitch:%f, roll:%f", info.fisheye_info[cam_id].extrinsic.yaw, info.fisheye_info[cam_id].extrinsic.pitch, info.fisheye_info[cam_id].extrinsic.roll);
                XCAM_LOG_INFO ("fisheye translation: x:%f, y:%f, z:%f", info.fisheye_info[cam_id].extrinsic.trans_x, info.fisheye_info[cam_id].extrinsic.trans_y, info.fisheye_info[cam_id].extrinsic.trans_z);
            }

            stitcher->set_stitch_info (info);
        } else {
            PointFloat3 camera_poss[XCAM_STITCH_MAX_CAMERAS];

            stitcher->set_intrinsic_names (intrinsic_names);
            stitcher->set_extrinsic_names (extrinsic_names);
            // 对于碗面模式，直接从文本文件读取标定参数并配置 Bowl 模型。
            //stitcher->set_bowl_config (bowl_config (cam_model));

            stitcher->init_camera_info (); // 提前提取camera info
            const uint32_t cam_num = stitcher->get_camera_num ();

            for (uint32_t i = 0; i < cam_num && i < XCAM_STITCH_MAX_CAMERAS; ++i) {
                CameraInfo cam_info;
                if (!stitcher->get_camera_info (i, cam_info)) {
                    XCAM_LOG_ERROR ("fail to get info for %dth camera\n", i);
                    continue;
                }

                const ExtrinsicParameter &extr = cam_info.calibration.extrinsic;
                camera_poss[i].x = extr.trans_x;
                camera_poss[i].y = extr.trans_y;
                camera_poss[i].z = extr.trans_z;
            }

            BowlDataConfig bowl = cal_bowl_config (camera_poss, cam_num, 600.0f, 400.0f);
            stitcher->set_bowl_config (bowl);
        }

        if (out_config.save_topview) {
            const uint32_t prev_out_size = outs.size();
            add_stream (outs, "topview", topview_width, topview_height);
            XCAM_ASSERT (outs.size() == prev_out_size + 1);

            out_config.topview_index = outs.size() - 1;

            CHECK (outs[out_config.topview_index]->estimate_file_format (),
                   "%s: estimate file format failed", outs[out_config.topview_index]->get_file_name ());
            CHECK (outs[out_config.topview_index]->open_writer ("wb"), "open output file(%s) failed", outs[out_config.topview_index]->get_file_name ());

            // 为顶视图输出配置独立的 GeoMapper，以便在主循环中直接 remap。
            create_topview_mapper (stitcher, outs[out_config.stitch_index], outs[out_config.topview_index], module);
        }

        if (out_config.save_cubemap) {
            const uint32_t prev_out_size = outs.size();
            add_stream (outs, "cubemap", cubemap_width, cubemap_height);
            XCAM_ASSERT (outs.size() == prev_out_size + 1);

            out_config.cubemap_index = outs.size() - 1;

            CHECK (outs[out_config.cubemap_index]->estimate_file_format (),
                   "%s: estimate file format failed", outs[out_config.cubemap_index]->get_file_name ());
            CHECK (outs[out_config.cubemap_index]->open_writer ("wb"), "open output file(%s) failed", outs[out_config.cubemap_index]->get_file_name ());

            // 为 Cubemap 输出创建映射，流程与顶视图类似。
            create_cubemap_mapper (stitcher, outs[out_config.stitch_index], outs[out_config.cubemap_index], module);
        }
        CHECK_EXP (
            run_stitcher (stitcher, ins, outs, frame_mode, out_config, loop, enable_dmabuf) == 0,
            "run stitcher failed");
    }

    return 0;
}
