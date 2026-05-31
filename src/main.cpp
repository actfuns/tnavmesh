#include "tnavmesh/commands.h"
#include "tnavmesh/version.h"
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage();
        return 1;
    }

    std::string cmd = argv[1];
    int subArgc = argc - 1;
    char** subArgv = argv + 1;  // subArgv[0] = command name

    if (cmd == "build")   return cmd_build(subArgc, subArgv);
    if (cmd == "inspect") return cmd_inspect(subArgc, subArgv);
    if (cmd == "path")    return cmd_path(subArgc, subArgv);
    if (cmd == "--version") {
        std::cout << TNAVMESH_VERSION << "\n";
        return 0;
    }
    if (cmd == "--help" || cmd == "-h") {
        printUsage();
        return 0;
    }

    std::cerr << "Unknown command: " << cmd << "\n";
    printUsage();
    return 1;
}
