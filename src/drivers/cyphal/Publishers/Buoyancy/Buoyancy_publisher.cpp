#include "Buoyancy_publisher.hpp"
#include <drivers/drv_hrt.h> // Required for hrt_absolute_time()

void BuoyancyPublisher::update()
{
    // 1. Check if the port ID is set (Parameter isn't -1)
    if (_port_id == CANARD_PORT_ID_UNSET) {
        return;
    }

    buoyancy_control_s uorb_data;

    // 2. Check for new internal uORB data
    if (_sub.update(&uorb_data)) {

        // 3. Prepare Cyphal message structure
        robosub_BuoyancyControl_1_0 cyphal_msg{};
        for (int i = 0; i < 4; i++) {
            cyphal_msg.tank_command[i] = uorb_data.tank_command[i];
        }

        // 4. Serialize (convert to bytes)
        uint8_t buffer[robosub_BuoyancyControl_1_0_EXTENT_BYTES_];
        size_t buffer_size = robosub_BuoyancyControl_1_0_EXTENT_BYTES_;

        if (robosub_BuoyancyControl_1_0_serialize_(&cyphal_msg, buffer, &buffer_size) >= 0) {

            // 5. Create the Metadata (The "Envelope" for the message)
            CanardTransferMetadata metadata{};
            metadata.priority       = CanardPriorityNominal;
            metadata.transfer_kind  = CanardTransferKindMessage;
            metadata.port_id        = _port_id;                // From base class
            metadata.remote_node_id = CANARD_NODE_ID_UNSET;    // Messages are broadcast
            metadata.transfer_id    = _transfer_id;            // From base class

            // 6. Set a deadline (Now + 100ms)
            const CanardMicrosecond deadline = hrt_absolute_time() + 100000;

            // 7. THE FIX: Push to the hardware queue
            if (_canard_handle.TxPush(deadline, &metadata, buffer_size, buffer) >= 0) {
                // Increment the transfer ID for the next message (required by Cyphal)
                _transfer_id++;
            }
        }
    }
}
