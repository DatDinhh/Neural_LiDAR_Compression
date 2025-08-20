// io/pcd_reader.cpp
#include "pcd_reader.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <array>
#include <algorithm>
#include <charconv>
#include <cstring>

namespace pcc::io {

namespace {

struct PcdHeader {
    std::vector<std::string> fields; 
    std::vector<int>         size;   
    std::vector<char>        type;   
    std::vector<int>         count;  
    int width = 0;
    int height = 1;
    int points = 0; 
    std::string data; 
    std::size_t header_bytes = 0; 

    int x_idx = -1, y_idx = -1, z_idx = -1;
    int point_step = 0; 
};

static inline std::string ltrim(std::string s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
        [](unsigned char c){ return !std::isspace(c); }));
    return s;
}
static inline std::string rtrim(std::string s) {
    s.erase(std::find_if(s.rbegin(), s.rend(),
        [](unsigned char c){ return !std::isspace(c); }).base(), s.end());
    return s;
}
static inline std::string trim(std::string s) { return rtrim(ltrim(std::move(s))); }

static std::vector<std::string> split_ws(const std::string& s) {
    std::istringstream is(s);
    std::vector<std::string> out;
    std::string tok;
    while (is >> tok) out.push_back(tok);
    return out;
}

static bool parse_header(std::istream& in, PcdHeader& h) {
    std::string line;
    std::size_t bytes=0;
    bool have_data=false;
    while (std::getline(in, line)) {
        bytes += line.size() + 1; // '\n'
        line = trim(line);
        if (line.empty()) continue;
        if (line[0] == '#') continue;

        auto pos = line.find(' ');
        std::string key = (pos==std::string::npos)? line : line.substr(0,pos);
        std::string val = (pos==std::string::npos)? ""  : trim(line.substr(pos+1));
        for (auto& c: key) c = static_cast<char>(std::toupper(c));

        if (key == "FIELDS") {
            h.fields = split_ws(val);
        } else if (key == "SIZE") {
            auto v = split_ws(val);
            h.size.resize((int)v.size());
            for (size_t i=0;i<v.size();++i) h.size[(int)i] = std::stoi(v[i]);
        } else if (key == "TYPE") {
            auto v = split_ws(val);
            h.type.resize((int)v.size());
            for (size_t i=0;i<v.size();++i) h.type[(int)i] = v[i].empty()? '?' : v[i][0];
        } else if (key == "COUNT") {
            auto v = split_ws(val);
            h.count.resize((int)v.size());
            for (size_t i=0;i<v.size();++i) h.count[(int)i] = std::stoi(v[i]);
        } else if (key == "WIDTH") {
            h.width = std::stoi(val);
        } else if (key == "HEIGHT") {
            h.height = std::stoi(val);
        } else if (key == "POINTS") {
            h.points = std::stoi(val);
        } else if (key == "DATA") {
            h.data = val;
            have_data = true;
            break;
        }
    }
    if (!have_data) return false;

    if (h.count.empty()) h.count.assign(h.fields.size(), 1);
    if (h.type.size()  != h.fields.size()) return false;
    if (h.size.size()  != h.fields.size()) return false;

    // compute point_step
    int step = 0;
    for (size_t i=0;i<h.fields.size();++i) {
        step += h.size[i] * h.count[i];
    }
    h.point_step = step;
    h.header_bytes = bytes;

    // locate x,y,z indices among scalars
    for (size_t i=0;i<h.fields.size();++i) {
        const std::string& f = h.fields[i];
        if (f=="x") h.x_idx = (int)i;
        else if (f=="y") h.y_idx = (int)i;
        else if (f=="z") h.z_idx = (int)i;
    }

    if (h.points == 0) {
        if (h.width == 0) return false;
        h.points = h.width * std::max(1, h.height);
    }
    return (h.x_idx>=0 && h.y_idx>=0 && h.z_idx>=0);
}

static inline float read_f32_le(const std::uint8_t* p) {
    float v;
    std::memcpy(&v, p, sizeof(float));
    return v;
}

}

bool read_pcd_positions(const std::string& path,
                        std::vector<float3>& out,
                        std::size_t max_points) {
    out.clear();

    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) return false;

    PcdHeader h;
    if (!parse_header(ifs, h)) return false;

    const int n = h.points;
    const std::size_t cap = (max_points>0)? std::min<std::size_t>(max_points, n)
                                          : static_cast<std::size_t>(n);
    out.reserve(cap);

    // offsets for x,y,z
    auto field_offset = [&](int idx) {
        int off = 0;
        for (int i=0;i<idx;++i) off += h.size[i]*h.count[i];
        return off;
    };
    const int off_x = field_offset(h.x_idx);
    const int off_y = field_offset(h.y_idx);
    const int off_z = field_offset(h.z_idx);

    if (h.data == "ascii") {
        std::string line;
        std::size_t count = 0;
        while (count < cap && std::getline(ifs, line)) {
            if (line.empty()) continue;
            std::istringstream ls(line);
            std::vector<double> cols; cols.reserve(h.fields.size());
            double v;
            while (ls >> v) cols.push_back(v);
            // require at least up to z index
            if ((int)cols.size() < std::max({h.x_idx,h.y_idx,h.z_idx})+1) continue;
            float3 p;
            p.x = static_cast<float>(cols[(size_t)h.x_idx]);
            p.y = static_cast<float>(cols[(size_t)h.y_idx]);
            p.z = static_cast<float>(cols[(size_t)h.z_idx]);
            out.push_back(p);
            ++count;
        }
        return !out.empty();
    }
    else if (h.data == "binary") {
        // Jump to payload start
        ifs.clear();
        ifs.seekg(static_cast<std::streamoff>(h.header_bytes), std::ios::beg);
        if (!ifs) return false;

        const std::size_t point_step = static_cast<std::size_t>(h.point_step);
        std::vector<std::uint8_t> buf(64 * 1024); // bounded scratch buffer
        std::size_t done = 0;

        while (done < cap) {
            const std::size_t pts_left = cap - done;
            const std::size_t bytes_to_read =
                std::min<std::size_t>(pts_left * point_step, buf.size());
            ifs.read(reinterpret_cast<char*>(buf.data()),
                     static_cast<std::streamsize>(bytes_to_read));
            const std::size_t got = static_cast<std::size_t>(ifs.gcount());
            if (got == 0) break;

            const std::size_t pts_in_chunk = got / point_step;
            const std::uint8_t* p = buf.data();
            for (std::size_t i = 0; i < pts_in_chunk; ++i, p += point_step) {
                float3 q;
                q.x = read_f32_le(p + off_x);
                q.y = read_f32_le(p + off_y);
                q.z = read_f32_le(p + off_z);
                out.push_back(q);
            }
            done += pts_in_chunk;
        }
        return !out.empty();
    }
    else {
        return false;
    }
}

bool write_pcd_ascii(const std::string& path,
                     std::span<const float3> pts) {
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) return false;

    const int width  = static_cast<int>(pts.size());
    const int height = 1;

    ofs << "# .PCD v0.7 - Point Cloud Data file format\n";
    ofs << "VERSION 0.7\n";
    ofs << "FIELDS x y z\n";
    ofs << "SIZE 4 4 4\n";
    ofs << "TYPE F F F\n";
    ofs << "COUNT 1 1 1\n";
    ofs << "WIDTH " << width << "\n";
    ofs << "HEIGHT " << height << "\n";
    ofs << "VIEWPOINT 0 0 0 1 0 0 0\n";
    ofs << "POINTS " << width*height << "\n";
    ofs << "DATA ascii\n";
    for (const auto& p : pts) {
        ofs << p.x << " " << p.y << " " << p.z << "\n";
    }
    return ofs.good();
}

} 
