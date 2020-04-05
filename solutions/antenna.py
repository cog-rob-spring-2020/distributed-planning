from uuid import uuid4
from pubsub import pub


TOPIC_BIDS = "bids"
TOPIC_ESTOPS = "estops"
TOPIC_WAYPOINTS = "waypoints"

class Antenna:
    def __init__(self):
        # unique identifier
        self.uuid = uuid4()
        self.peers = []

        # message callbacks
        self.handlers = {}

        pub.subscribe(self, TOPIC_BIDS)
        pub.subscribe(self, TOPIC_ESTOPS)
        pub.subscribe(self, TOPIC_WAYPOINTS)

    def __call__(self, data):
        """
        Receives messages from peers
        """
        for cb in self.handlers[data.topic]:
            cb(data.msg)

    def find_peers(self);
        pass

    def broadcast(self, topic, msg):
        pub.sendMessage(topic, self.uuid, msg)

    def on_message(self, topic, handler):
        if not topic in self.handlers:
            self.handlers[topic] = []
        self.handlers[topic].append(handler)
