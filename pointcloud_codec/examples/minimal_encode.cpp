#include <draco/compression/encode.h>
#include <draco/compression/decode.h>
#include <draco/core/encoder_buffer.h>
#include <draco/core/decoder_buffer.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>
#include <draco/attributes/geometry_attribute.h>

#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using PointT = pcl::PointXYZ;

static bool load_xyz_with_pcl(const std::string& path, std::vector<float>& xyz) {
    pcl::PCLPointCloud2 blob;
    if (pcl::io::loadPCDFile(path, blob) != 0) { std::cerr << "PCD load failed: " << path << "\n"; return false; }
    pcl::PointCloud<PointT> cloud;
    pcl::fromPCLPointCloud2(blob, cloud);
    if (cloud.empty()) { std::cerr << "PCD has zero points: " << path << "\n"; return false; }
    xyz.resize(static_cast<size_t>(cloud.size())*3);
    for (size_t i=0;i<cloud.size();++i){ xyz[3*i+0]=cloud[i].x; xyz[3*i+1]=cloud[i].y; xyz[3*i+2]=cloud[i].z; }
    return true;
}

static bool save_xyz_with_pcl(const std::string& path, const std::vector<float>& xyz, bool binary=true) {
    if (xyz.size()%3!=0) return false;
    pcl::PointCloud<PointT> cloud;
    cloud.resize(xyz.size()/3);
    for (size_t i=0;i<cloud.size();++i){ cloud[i].x=xyz[3*i+0]; cloud[i].y=xyz[3*i+1]; cloud[i].z=xyz[3*i+2]; }
    int rc = binary ? pcl::io::savePCDFileBinary(path, cloud) : pcl::io::savePCDFileASCII(path, cloud);
    if (rc!=0) std::cerr << "PCD save failed: " << path << "\n";
    return rc==0;
}

static bool draco_encode_xyz(const std::vector<float>& xyz, int qpos_bits, std::string& out_bytes) {
    const int n = static_cast<int>(xyz.size()/3);
    if (n<=0) { std::cerr << "encode: no points\n"; return false; }
    draco::PointCloudBuilder b; b.Start(n);
    const int pos = b.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
    if (pos<0) { std::cerr << "encode: AddAttribute failed\n"; return false; }
    b.SetAttributeValuesForAllPoints(pos, xyz.data(), sizeof(float)*3);
    std::unique_ptr<draco::PointCloud> pc = b.Finalize(false);
    if (!pc) { std::cerr << "encode: Finalize failed\n"; return false; }
    draco::Encoder enc;
    if (qpos_bits>0) enc.SetAttributeQuantization(draco::GeometryAttribute::POSITION, qpos_bits);
    draco::EncoderBuffer buf;
    const draco::Status st = enc.EncodePointCloudToBuffer(*pc, &buf);
    if (!st.ok()) { std::cerr << "encode: " << st.error_msg_string() << "\n"; return false; }
    out_bytes.assign(buf.data(), buf.data()+buf.size());
    return true;
}
static bool draco_decode_xyz(const std::string& bytes, std::vector<float>& xyz) {
    draco::DecoderBuffer dbuf; dbuf.Init(bytes.data(), bytes.size());
    draco::Decoder dec;
    auto so = dec.DecodePointCloudFromBuffer(&dbuf);
    if (!so.ok()) { std::cerr << "decode: " << so.status().error_msg_string() << "\n"; return false; }
    std::unique_ptr<draco::PointCloud> pc = std::move(so).value();
    const draco::PointAttribute* pa = pc->GetNamedAttribute(draco::GeometryAttribute::POSITION);
    if (!pa || pa->num_components()!=3) { std::cerr << "decode: POSITION missing/invalid\n"; return false; }
    xyz.resize(static_cast<size_t>(pc->num_points())*3);
    for (draco::PointIndex i(0); i<pc->num_points(); ++i){
        float v[3]; const auto avi = pa->mapped_index(i); pa->GetValue(avi, v);
        xyz[3*i.value()+0]=v[0]; xyz[3*i.value()+1]=v[1]; xyz[3*i.value()+2]=v[2];
    }
    return true;
}

static void usage(){
    std::cout <<
      "minimal_encode.exe   PCD<->DRC tool (PCL I/O)\n"
      "Encode: minimal_encode.exe encode -i in.pcd -o out.drc --qpos-bits N\n"
      "Decode: minimal_encode.exe decode -i in.drc -o out.pcd\n";
}
static uintmax_t fsize(const std::string& p){
    std::error_code ec; auto s=std::filesystem::file_size(p,ec); return ec?0:s;
}

int main(int argc,char** argv){
    if (argc<2) { usage(); return 0; }
    std::string mode=argv[1], in, out; int q=14;
    for (int i=2;i<argc;++i){
        std::string a=argv[i];
        auto next=[&]{ if (i+1>=argc){ std::cerr<<"missing value after "<<a<<"\n"; std::exit(2);} return std::string(argv[++i]); };
        if (a=="-i"||a=="--input") in=next();
        else if (a=="-o"||a=="--output") out=next();
        else if (a=="--qpos-bits"||a=="-q") q=std::atoi(next().c_str());
        else if (a=="--help"||a=="-h"||a=="/?") { usage(); return 0; }
        else { std::cerr<<"unknown arg: "<<a<<"\n"; usage(); return 2; }
    }
    if (mode=="encode"){
        if (in.empty()||out.empty()){ usage(); return 2; }
        std::vector<float> xyz; if (!load_xyz_with_pcl(in,xyz)) return 1;
        std::string bytes; if (!draco_encode_xyz(xyz,q,bytes)) return 1;
        std::ofstream f(out, std::ios::binary); f.write(bytes.data(), bytes.size());
        std::cout<<"Encoded "<<(xyz.size()/3)<<" pts -> "<<bytes.size()<<" bytes (q="<<q<<")\n";
        std::cout<<"Original PCD ~"<<fsize(in)<<" bytes; ratio   "
                 <<std::fixed<<std::setprecision(2)
                 <<(fsize(in)? (double)bytes.size()/ (double)fsize(in) : 0.0)<<"x\n";
        return 0;
    } else if (mode=="decode"){
        if (in.empty()||out.empty()){ usage(); return 2; }
        std::ifstream f(in, std::ios::binary); if (!f){ std::cerr<<"cannot open "<<in<<"\n"; return 1; }
        std::string bytes((std::istreambuf_iterator<char>(f)), {});
        std::vector<float> xyz; if (!draco_decode_xyz(bytes,xyz)) return 1;
        if (!save_xyz_with_pcl(out, xyz, true)) return 1;
        std::cout<<"Decoded "<<(xyz.size()/3)<<" pts -> "<<out<<"\n";
        return 0;
    } else { usage(); return 2; }
}
