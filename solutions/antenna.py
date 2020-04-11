from uuid import uuid4
from pubsub import pub

TOPIC_BIDS = "bids"
TOPIC_WAYPOINTS = "waypoints"
TOPIC_PEERS = "peers"

class Antenna:
    def __init__(self):
        # unique identifier
        self.uuid = uuid4()

        # message callbacks
        self.handlers = {}

        pub.subscribe(self, TOPIC_BIDS)
        pub.subscribe(self, TOPIC_WAYPOINTS)
        pub.subscribe(self, TOPIC_PEERS)

    def __call__(self, sender_id, msg):
        """
        Receives messages from peers
        """
        topic = msg["topic"]
        for callback in self.handlers[topic]:
            callback(sender_id, msg)

    def broadcast(self, topic, msg):
        # naming the arguments sender_id and msg for now because PyPubSub requires all non-topic arguments to be named
        pub.sendMessage(topic, sender_id = self.uuid, msg = msg)

    def on_message(self, topic, handler):
        if not topic in self.handlers:
            self.handlers[topic] = []
        self.handlers[topic].append(handler)
