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
    void logDebug(Args... argument) {
        std::stringstream msg;
        (msg << ... << argument);
        KDTreeLogger::defaultLogger().getLogger()->debug(msg.str());
    }

    template<typename... Args>
    void logInfo(Args... argument) {
        std::stringstream msg;
        (msg << ... << argument);
        KDTreeLogger::defaultLogger().getLogger()->info(msg.str());
    }

    template<typename... Args>
    void logWarn(Args... argument) {
        std::stringstream msg;
        (msg << ... << argument);
        KDTreeLogger::defaultLogger().getLogger()->warn(msg.str());
    }

    template<typename... Args>
    void logError(Args... argument) {
        std::stringstream msg;
        (msg << ... << argument);
        KDTreeLogger::defaultLogger().getLogger()->error(msg.str());
    }

}// namespace kdtree

// macros so the args (string concat, to_string, ...) aren't even evaluated when the level is off,
// a plain function still builds the whole message on every call
// do-while(0) so the macro eats the trailing ; and works in an unbraced if/else
#define KDTREE_LOG_IF_ENABLED(level, func, ...)                                       \
    do {                                                                              \
        if (::kdtree::KDTreeLogger::defaultLogger().getLogger()->should_log(level)) { \
            func(__VA_ARGS__);                                                        \
        }                                                                             \
    } while (0)

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_DEBUG
    #define LOG_DEBUG(...) KDTREE_LOG_IF_ENABLED(spdlog::level::debug, ::kdtree::logDebug, __VA_ARGS__)
#else
    #define LOG_DEBUG(...) ((void) 0)
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_INFO
    #define LOG_INFO(...) KDTREE_LOG_IF_ENABLED(spdlog::level::info, ::kdtree::logInfo, __VA_ARGS__)
#else
    #define LOG_INFO(...) ((void) 0)
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_WARN
    #define LOG_WARN(...) KDTREE_LOG_IF_ENABLED(spdlog::level::warn, ::kdtree::logWarn, __VA_ARGS__)
#else
    #define LOG_WARN(...) ((void) 0)
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_ERROR
    #define LOG_ERROR(...) KDTREE_LOG_IF_ENABLED(spdlog::level::err, ::kdtree::logError, __VA_ARGS__)
#else
    #define LOG_ERROR(...) ((void) 0)
#endif
