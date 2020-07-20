import natnetclient as natnet
client = natnet.NatClient(client_ip='192.168.1.101', data_port=1511, comm_port=1510)

hand = client.rigid_bodies['Rigid Body1'] # Assuming a Motive Rigid Body is available that you named "Hand"
print(hand.position)
print(hand.rotation)

hand_markers = hand.markers  # returns a list of markers, each with their own properties
print(hand_markers[0].position)