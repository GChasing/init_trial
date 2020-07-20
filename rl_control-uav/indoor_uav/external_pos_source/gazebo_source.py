import pygazebo
from pygazebo.msg import packet_pb2
from pygazebo.msg import subscribe_pb2
import trollius
from trollius import From

@trollius.coroutine
def subscribe_test():
    # Nothing beyond the base fixture is required for this test.
    manager = yield From(pygazebo.connect())
    print('Pygazebo connected.')
    received_data = []
    first_data_future = trollius.Future()

    def callback(data):
        received_data.append(data)
        if not first_data_future.done():
            first_data_future.set_result(None)

    listen = trollius.Future()

    manager.server.read_packet(lambda data: listen.set_result(data))
    subscriber = manager.manager.subscribe(
        'subscribetopic', 'othermsgtype', callback)
    assert subscriber is not None
    print('Before run listen')
    loop.run_until_complete(listen)
    print('After run listen')
    packet_data = listen.result()

    # We should have received a subscribe for this topic.
    packet = packet_pb2.Packet.FromString(packet_data)
    assert packet.type == 'subscribe'

    subscribe = subscribe_pb2.Subscribe.FromString(
        packet.serialized_data)
    assert subscribe.topic == 'subscribetopic'
    assert subscribe.msg_type == 'othermsgtype'

loop = trollius.get_event_loop()
loop.run_until_complete(subscribe_test())