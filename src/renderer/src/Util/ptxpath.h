#ifndef PTXPATH_H
#define PTXPATH_H

#include "sampleConfig.h"

inline std::string ptxpath() {
  std::string ptx_dir = PTX_DIR;
  return std::string(ptx_dir + "/cuda_compile_ptx_generated_");
}

#endif /* ifndef PTXPATH_H */
