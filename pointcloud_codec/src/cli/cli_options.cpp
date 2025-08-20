// Implementation of parse() declared in cli_options.hpp
#include "cli_options.hpp"
#include <sstream>
#include <cstdlib>

namespace pcc::cli {

static std::string help_text() {
    std::ostringstream os;
    os <<
"pcc - Lidar point cloud codec\n"
"\n"
"Usage:\n"
"  pcc encode <in.pcd> <out.drc> [--qpos B] [--chunk K]\n"
"  pcc decode <in.drc> <out.pcd>\n"
"  pcc bench  <in.pcd> --qpos 10,12,14,16 --report out.csv [--chunk K]\n"
"\n";
    return os.str();
}

#if PCC_HAVE_CXXOPTS
static std::vector<int> split_ints(const std::string& s) {
    std::vector<int> out;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (!tok.empty()) out.push_back(std::atoi(tok.c_str()));
    }
    return out;
}
#endif

Parsed parse(int argc, char** argv) {
    Parsed p;
    if (argc < 2) { p.mode = Mode::Help; p.help = help_text(); return p; }

    const std::string cmd = argv[1];
    if (cmd == "encode") {
#if PCC_HAVE_CXXOPTS
        cxxopts::Options options("pcc encode", "Encode PCD into drc stream");
        options.add_options()
            ("qpos", "Position quantization bits", cxxopts::value<int>()->default_value("14"))
            ("chunk","Chunk size", cxxopts::value<int>()->default_value("65536"))
            ;
        options.add_options("positional")
            ("in",  "Input PCD",  cxxopts::value<std::string>())
            ("out", "Output DRC", cxxopts::value<std::string>());
        options.parse_positional({"in","out"});

        auto result = options.parse(argc, argv);
        if (!result.count("in") || !result.count("out")) {
            p.mode = Mode::Help; p.help = help_text(); return p;
        }
        CmdEncode e;
        e.in_path = result["in"].as<std::string>();
        e.out_path= result["out"].as<std::string>();
        e.qpos_bits = result["qpos"].as<int>();
        e.chunk_points = static_cast<std::size_t>(result["chunk"].as<int>());
        p.mode = Mode::Encode;
        p.enc = e;
        return p;
#else
        if (argc < 4) { p.mode = Mode::Help; p.help = help_text(); return p; }
        CmdEncode e;
        e.in_path  = argv[2];
        e.out_path = argv[3];
        // naive flags
        for (int i=4;i<argc;++i) {
            std::string a = argv[i];
            if (a == "--qpos" && i+1<argc) e.qpos_bits = std::atoi(argv[++i]);
            else if (a == "--chunk" && i+1<argc) e.chunk_points = static_cast<std::size_t>(std::atoi(argv[++i]));
        }
        p.mode = Mode::Encode; p.enc = e; return p;
#endif
    }
    else if (cmd == "decode") {
#if PCC_HAVE_CXXOPTS
        cxxopts::Options options("pcc decode", "Decode drc stream into PCD");
        options.add_options("positional")
            ("in",  "Input DRC",  cxxopts::value<std::string>())
            ("out", "Output PCD", cxxopts::value<std::string>());
        options.parse_positional({"in","out"});
        auto result = options.parse(argc, argv);
        if (!result.count("in") || !result.count("out")) {
            p.mode = Mode::Help; p.help = help_text(); return p;
        }
        CmdDecode d;
        d.in_path  = result["in"].as<std::string>();
        d.out_path = result["out"].as<std::string>();
        p.mode = Mode::Decode; p.dec = d; return p;
#else
        if (argc < 4) { p.mode = Mode::Help; p.help = help_text(); return p; }
        CmdDecode d; d.in_path = argv[2]; d.out_path = argv[3];
        p.mode = Mode::Decode; p.dec = d; return p;
#endif
    }
    else if (cmd == "bench") {
#if PCC_HAVE_CXXOPTS
        cxxopts::Options options("pcc bench", "Sweep qpos and report metrics");
        options.add_options()
            ("qpos", "Comma-separated bits (e.g. 10,12,14)",
                cxxopts::value<std::string>())
            ("report","CSV path", cxxopts::value<std::string>())
            ("chunk","Chunk size", cxxopts::value<int>()->default_value("65536"))
            ;
        options.add_options("positional")
            ("in", "Input PCD", cxxopts::value<std::string>());
        options.parse_positional({"in"});

        auto result = options.parse(argc, argv);
        if (!result.count("in") || !result.count("qpos") || !result.count("report")) {
            p.mode = Mode::Help; p.help = help_text(); return p;
        }
        CmdBench b;
        b.in_path = result["in"].as<std::string>();
        b.qpos_list = split_ints(result["qpos"].as<std::string>());
        b.report_csv= result["report"].as<std::string>();
        b.chunk_points = static_cast<std::size_t>(result["chunk"].as<int>());
        p.mode = Mode::Bench; p.bench = b; return p;
#else
        if (argc < 4) { p.mode = Mode::Help; p.help = help_text(); return p; }
        CmdBench b;
        b.in_path = argv[2];
        for (int i=3;i<argc;++i) {
            std::string a = argv[i];
            if (a == "--qpos" && i+1<argc) {
                std::string s = argv[++i];
                std::stringstream ss(s); std::string tok;
                while (std::getline(ss,tok,',')) b.qpos_list.push_back(std::atoi(tok.c_str()));
            } else if (a == "--report" && i+1<argc) {
                b.report_csv = argv[++i];
            } else if (a == "--chunk" && i+1<argc) {
                b.chunk_points = static_cast<std::size_t>(std::atoi(argv[++i]));
            }
        }
        if (b.qpos_list.empty() || b.report_csv.empty()) {
            p.mode=Mode::Help; p.help=help_text(); return p;
        }
        p.mode = Mode::Bench; p.bench = b; return p;
#endif
    }

    p.mode = Mode::Help; p.help = help_text(); return p;
}

} 
