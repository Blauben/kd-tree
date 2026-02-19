#pragma once

#include <memory>
#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>


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

    inline void DEBUG(std::string_view msg) {
        #ifdef DEBUG
        KDTreeLogger::DEFAULT_LOGGER.getLogger()->debug(msg);
        #endif
    }

    inline void INFO(std::string_view msg) {
        KDTreeLogger::DEFAULT_LOGGER.getLogger()->info(msg);
    }

    inline void WARN(std::string_view msg) {
        KDTreeLogger::DEFAULT_LOGGER.getLogger()->warn(msg);
    }

    inline void ERROR(std::string_view msg) {
        KDTreeLogger::DEFAULT_LOGGER.getLogger()->error(msg);
    }

}