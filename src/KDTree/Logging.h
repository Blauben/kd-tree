#pragma once

#include <memory>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <sstream>

namespace kdtree {

    class KDTreeLogger {
    private:
        const std::shared_ptr<spdlog::logger> _logger;

    public:
        KDTreeLogger()
            : _logger(spdlog::stdout_color_mt<spdlog::synchronous_factory>("KDTREE_LOGGER")) {
            _logger->set_level(static_cast<spdlog::level::level_enum>(SPDLOG_ACTIVE_LEVEL));
        }

        [[nodiscard]] inline std::shared_ptr<spdlog::logger> getLogger() const {
            return _logger;
        }

        // function-local singleton: constructed on first use, avoids static-init order problems
        static KDTreeLogger &defaultLogger() {
            static KDTreeLogger instance;
            return instance;
        }
    };

    template<typename... Args>
    void LOG_DEBUG(Args... argument) {
        std::stringstream msg;
        (msg << ... << argument);
        KDTreeLogger::defaultLogger().getLogger()->debug(msg.str());
    }

    template<typename... Args>
    void LOG_INFO(Args... argument) {
        std::stringstream msg;
        (msg << ... << argument);
        KDTreeLogger::defaultLogger().getLogger()->info(msg.str());
    }

    template<typename... Args>
    void LOG_WARN(Args... argument) {
        std::stringstream msg;
        (msg << ... << argument);
        KDTreeLogger::defaultLogger().getLogger()->warn(msg.str());
    }

    template<typename... Args>
    void LOG_ERROR(Args... argument) {
        std::stringstream msg;
        (msg << ... << argument);
        KDTreeLogger::defaultLogger().getLogger()->error(msg.str());
    }

}// namespace kdtree