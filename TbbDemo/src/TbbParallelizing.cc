#include "TbbParallelizing.h"

namespace HugoLiuGithub
{
    std::atomic<size_t> TbbParallelizing::ParallelWrapper_1::counter_ {0};
    std::atomic<size_t> TbbParallelizing::ParallelWrapper_2::counter_ {0};
}