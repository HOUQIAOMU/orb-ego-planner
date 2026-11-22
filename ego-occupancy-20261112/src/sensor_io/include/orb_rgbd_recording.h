#pragma once
#include "orb_live_common.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <memory>

namespace orb_live {
using SteadyClock = std::chrono::steady_clock;
struct RGBDFrame {
    cv::Mat color, depth; // owned BGR8 / aligned Z16; immutable after publication
    double color_sec = 0, depth_sec = 0, valid_depth = 0;
    std::uint64_t color_number = 0, depth_number = 0, index = 0;
    int color_domain = 0, depth_domain = 0;
    SteadyClock::time_point received, enqueued;
};
using FramePtr = std::shared_ptr<const RGBDFrame>;

// One disk thread owns this object. Associations are the commit manifest.
class SequenceWriter {
public:
    explicit SequenceWriter(const std::filesystem::path& root) : root_(root) {
        namespace fs = std::filesystem;
        for (const auto* file : {"rgb", "depth", "associations.txt", "frames.csv"})
            if (fs::exists(root / file)) throw std::runtime_error("Refusing to overwrite recorded data");
        fs::create_directories(root / "rgb"); fs::create_directory(root / "depth");
        associations_.exceptions(std::ios::failbit | std::ios::badbit);
        frames_.exceptions(std::ios::failbit | std::ios::badbit);
        associations_.open(root / "associations.txt"); frames_.open(root / "frames.csv");
        associations_ << std::fixed << std::setprecision(9);
        frames_ << "index,color_timestamp_sec,depth_timestamp_sec,color_number,depth_number,color_domain,depth_domain,valid_depth_ratio\n"
                << std::fixed << std::setprecision(9);
        frames_.flush();
    }
    void write(const RGBDFrame& p) {
        namespace fs = std::filesystem;
        if (failed_) throw std::runtime_error("Recording writer stopped after previous commit error");
        if (p.color.empty() || p.color.type()!=CV_8UC3 || p.depth.type()!=CV_16UC1 || p.color.size()!=p.depth.size())
            throw std::runtime_error("Recorder requires matched owned BGR8/Z16 images");
        std::ostringstream stem; stem << std::setw(6) << std::setfill('0') << count_;
        const std::string name=stem.str()+".png";
        const auto rgb=root_/"rgb"/name, depth=root_/"depth"/name;
        const auto rgb_tmp=root_/"rgb"/(stem.str()+".partial.png"), depth_tmp=root_/"depth"/(stem.str()+".partial.png");
        bool rgb_committed=false, depth_committed=false;
        const auto frame_offset=static_cast<std::streamoff>(frames_.tellp());
        const auto association_offset=static_cast<std::streamoff>(associations_.tellp());
        try {
            const std::vector<int> compression{cv::IMWRITE_PNG_COMPRESSION,0};
            if (!cv::imwrite(rgb_tmp.string(),p.color,compression) || !cv::imwrite(depth_tmp.string(),p.depth,compression))
                throw std::runtime_error("PNG write failed: "+root_.string());
            fs::rename(rgb_tmp,rgb); rgb_committed=true;
            fs::rename(depth_tmp,depth); depth_committed=true;
            frames_ << count_ << ',' << p.color_sec << ',' << p.depth_sec << ',' << p.color_number << ',' << p.depth_number
                    << ',' << p.color_domain << ',' << p.depth_domain << ',' << p.valid_depth << '\n';
            frames_.flush();
            associations_ << p.color_sec << " rgb/" << name << ' ' << p.depth_sec << " depth/" << name << '\n';
            associations_.flush();
            ++count_;
        } catch (...) {
            failed_=true;
            // Close first: delayed stream buffers must not append after rollback.
            frames_.exceptions(std::ios::goodbit); associations_.exceptions(std::ios::goodbit);
            frames_.close(); associations_.close();
            std::error_code frame_error, association_error;
            fs::resize_file(root_/"frames.csv",frame_offset,frame_error);
            fs::resize_file(root_/"associations.txt",association_offset,association_error);
            std::error_code ignored;
            fs::remove(rgb_tmp,ignored); fs::remove(depth_tmp,ignored);
            if(frame_error || association_error)
                throw std::runtime_error("Manifest rollback failed; retaining pair images for recovery: "+root_.string());
            // Never remove a path that this writer did not successfully commit.
            if(rgb_committed) fs::remove(rgb,ignored);
            if(depth_committed) fs::remove(depth,ignored);
            throw;
        }
    }
    std::uint64_t count() const { return count_; }
private:
    std::filesystem::path root_;
    std::ofstream associations_, frames_;
    std::uint64_t count_=0;
    bool failed_=false;
};
} // namespace orb_live
