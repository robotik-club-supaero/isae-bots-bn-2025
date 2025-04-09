#ifndef _ROS_QOS_RELIABILITY_HPP_
#define _ROS_QOS_RELIABILITY_HPP_

/// See ROS2 quality of service
/// @see https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
enum QosReliability {
    /// Requires that the publisher use a reliable QoS
    ReliableOnly,
    /// Allows (but not requires) the publisher to use a "best effort" QoS.
    /// The actual QoS to be used is implementation dependent.
    AllowBestEffort,
};

#endif