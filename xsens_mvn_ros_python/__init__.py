"""Python rewrite of the Xsens UDP parsing pipeline."""

from .client import MAX_MVN_DATAGRAM_SIZE, XSensUDPClient
from .protocol import (
    AngularSegmentKinematicsDatagram,
    CenterOfMassDatagram,
    JointAnglesDatagram,
    LinearSegmentKinematicsDatagram,
    ParserManager,
    QuaternionDatagram,
    StreamingProtocol,
    parse_datagram,
)

__all__ = [
    "AngularSegmentKinematicsDatagram",
    "CenterOfMassDatagram",
    "JointAnglesDatagram",
    "LinearSegmentKinematicsDatagram",
    "MAX_MVN_DATAGRAM_SIZE",
    "ParserManager",
    "QuaternionDatagram",
    "StreamingProtocol",
    "XSensUDPClient",
    "parse_datagram",
]
