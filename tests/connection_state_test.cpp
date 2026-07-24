#include <cassert>
#include <cstdint>

#include "../gattserver_connection_state.hpp"

int main()
{
    GattConnectionState state;
    assert(state.connectionHandle() == GattConnectionState::NO_CONNECTION);
    assert(state.mtu() == 23);
    assert(!state.encrypted());

    state.connected(7, false);
    assert(state.connectionHandle() == 7);
    assert(state.mtu() == 23);
    assert(!state.encrypted());

    state.mtuChanged(8, 100);
    assert(state.mtu() == 23);
    state.mtuChanged(7, 185);
    assert(state.mtu() == 185);

    state.encryptionChanged(8, true);
    assert(!state.encrypted());
    state.encryptionChanged(7, true);
    assert(state.encrypted());

    state.disconnected(8);
    assert(state.connectionHandle() == 7);
    state.disconnected(7);
    assert(state.connectionHandle() == GattConnectionState::NO_CONNECTION);
    assert(state.mtu() == 23);
    assert(!state.encrypted());

    state.connected(9, true);
    state.reset();
    assert(state.connectionHandle() == GattConnectionState::NO_CONNECTION);
    assert(state.mtu() == 23);
    assert(!state.encrypted());
    return 0;
}
