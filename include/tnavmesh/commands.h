#ifndef COMMANDS_H
#define COMMANDS_H

// Each command function receives (argc, argv) where argv[0] is the subcommand name.
// Returns exit code (0=success, 1=build error, 2=input error).

int cmd_build(int argc, char** argv);
int cmd_inspect(int argc, char** argv);
int cmd_path(int argc, char** argv);

void printUsage();

#endif
