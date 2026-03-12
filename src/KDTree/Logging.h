#pragma once

#include <memory>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <sstream>


namespace kdtree {

    class KDTreeLogger;

    /**
     * Wrapper Class for spdlog logger
     */
    class KDTreeLogger {

    public:
        /**
         * The default logger which is used in the whole implementation for every logging.
         */
        const static KDTreeLogger DEFAULT_LOGGER;

    private:
        /**
         * The actual spdlog::logger
         */
        const std::shared_ptr<spdlog::logger> _logger;

    public:
        /**
         * Constructs a new KDTreeLogger. Further, it registers the new logger in  spdlog's registry with
         * the name KDTREE_LOGGER.
         */
        KDTreeLogger()
            : _logger(spdlog::stdout_color_mt<spdlog::synchronous_factory>("KDTREE_LOGGER")) {
            _logger->set_level(spdlog::level::trace);
        }

        [[nodiscard]] inline std::shared_ptr<spdlog::logger> getLogger() const {
            return _logger;
        }
    };

    template<typename... Args>
    inline void LOG_DEBUG(Args... argument) {
#ifdef DEBUG
        std::stringstream msg;
        (msg << ... << argument);
        KDTreeLogger::DEFAULT_LOGGER.getLogger()->debug(msg.str());
#endif
    }

    template<typename... Args>
    inline void LOG_INFO(Args... argument) {
        std::stringstream msg;
        (msg << ... << argument);
        KDTreeLogger::DEFAULT_LOGGER.getLogger()->info(msg.str());
    }

    template<typename... Args>
    inline void LOG_WARN(Args... argument) {
        std::stringstream msg;
        (msg << ... << argument);
        KDTreeLogger::DEFAULT_LOGGER.getLogger()->warn(msg.str());
    }

    template<typename... Args>
    inline void LOG_ERROR(Args... argument) {
        std::stringstream msg;
        (msg << ... << argument);
        KDTreeLogger::DEFAULT_LOGGER.getLogger()->error(msg.str());
    }

}// namespace kdtree