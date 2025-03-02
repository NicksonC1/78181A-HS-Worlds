#pragma once

#include "genesis/logger/message.hpp"
#include "genesis/logger/baseSink.hpp"

namespace genesis {
/**
 * @brief Sink for sending messages to the terminal.
 *
 * This is the primary way of interacting with genesis's logging implementation. This sink is used for printing useful
 * information to the user's terminal.
 * <h3> Example Usage </h3>
 * @code
 * genesis::infoSink()->setLowestLevel(genesis::Level::INFO);
 * genesis::infoSink()->info("info: {}!", "my cool info here");
 * // Specify the order or placeholders
 * genesis::infoSink()->debug("{1} {0}!","world", "hello");
 * // Specify the precision of floating point numbers
 * genesis::infoSink()->warn("Thingy exceeded value: {:.2f}!", 93.1234);
 * @endcode
 */
class InfoSink : public BaseSink {
    public:
        /**
         * @brief Construct a new Info Sink object
         */
        InfoSink();
    private:
        /**
         * @brief Log the given message
         *
         * @param message
         */
        void sendMessage(const Message& message) override;
};
} // namespace genesis
