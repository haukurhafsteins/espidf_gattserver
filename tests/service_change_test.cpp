#include <cassert>
#include <cstdint>

#include "../gattserver_service_change.hpp"

int main()
{
    GattServiceChangeState state;
    uint16_t start = 0;
    uint16_t end = 0;

    assert(!state.applied());
    assert(!state.pending(start, end));
    assert(!state.schedule(0, 0xffff));
    assert(!state.schedule(1, 0));
    assert(!state.schedule(8, 7));

    assert(state.schedule(1, 0xffff));
    assert(state.pending(start, end));
    assert(start == 1);
    assert(end == 0xffff);
    assert(!state.schedule(2, 3));

    state.markApplied();
    assert(state.applied());
    assert(!state.pending(start, end));

    state.reset();
    assert(!state.applied());
    assert(!state.pending(start, end));
    assert(state.schedule(4, 9));
    assert(state.pending(start, end));
    assert(start == 4);
    assert(end == 9);
    return 0;
}
