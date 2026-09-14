#include "TPUart/Interface/Abstract.h"

namespace TPUart
{
namespace Interface
{

Abstract::~Abstract() = default;

// Wer keinen Überlauf melden kann, meldet keinen. Nicht rein virtuell, damit ein Interface ohne diese
// Information nicht gezwungen ist, eine Attrappe zu implementieren.
bool Abstract::overflow()
{
    return false;
}

} // namespace Interface
} // namespace TPUart
