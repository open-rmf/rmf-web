## api_server vs rest_server

The api server is a socketio server, its main goal is to record, cache and publish rmf events to a client browser. It is mainly used for streaming and server push.

The rest server on the other hand, will be a regular rest server it's main function is a one-shot request->response flow.

There are some key points that must be taken into account to ensure that they are scalable.

* There should only be 1 instance of api server running.
* All rooms in the api server must support "sticky" messages, or "transient local" in ros terms. It means that when a client subscribe to a room, the server must send the current state immediately. E.g., for a door state room, the server will send the last known door state of all doors upon subscription.
  * This allows us to scale the server via a "fan out" approach, proxy servers can be spun up on demand that connects to the main server to forward all the events.
  * This ensures event consistency. Some events are based off of other events, e.g. door health monitors a heartbeat on door states, if we let multiple servers listen on ros messages independently, they may produce different sets of events, e.g. one of the server may receive a door state slightly later than another (because of network latencies or other problems), then we have a conflicting state where some servers determine that a door has no heartbeat, while others determine that it is working fine. A malfunctioning equipment is a very important event so such conflicts will hinder the integrity of the system.
* The rest server should not subscribe to ros2 topics. If it needs access to the ros2 messages, it should get it from the api server. Publishing to ros2 is ok.
