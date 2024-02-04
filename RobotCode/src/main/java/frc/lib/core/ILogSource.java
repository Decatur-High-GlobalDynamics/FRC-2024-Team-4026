package frc.lib.core;

import java.text.MessageFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.logging.ConsoleHandler;
import java.util.logging.Filter;
import java.util.logging.Formatter;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

public interface ILogSource
{

    int STACK_TRACE_LENGTH = 5;

    /**
     * Using a map is hacky, but works. I'm not that concerned since map access is 0(1) time
     * complexity in most cases. If we have too many loggers, it could be a problem (worst case is
     * O(n) time complexity).
     */
    HashMap<String, Logger> Loggers = new HashMap<>();

    Formatter Formatter = new Formatter()
    {
        @Override
        public String format(LogRecord record)
        {
            return MessageFormat.format("{0,date,HH:mm:ss} {1} [{2}]: {3}\n", new Object[]
            {
                    new Date(record.getMillis()), record.getLoggerName(),
                    record.getLevel().getName(), record.getMessage()
            });
        }
    };

    class LogFilter implements Filter
    {
        private Level level;

        public LogFilter(Level level)
        {
            this.level = level;
        }

        public void setLevel(Level level)
        {
            this.level = level;
        }

        /** Determines whether the message is logged */
        @Override
        public boolean isLoggable(LogRecord record)
        {
            return record.getLevel().intValue() >= level.intValue();
        }
    }

    public default String getLoggerName()
    {
        return getClass().getName() + "@" + System.identityHashCode(this);
    }

    /**
     * Get the logger for this class. If the logger does not exist, it will be created.
     */
    private Logger getLogger()
    {
        String loggerName = getLoggerName();

        Logger logger = Loggers.get(loggerName);

        if (logger == null)
        {
            // Initialize the logger

            logger = Logger.getLogger(loggerName);
            logger.setLevel(Level.ALL); // We will filter the messages in our own filter

            // Set up the console handler
            ConsoleHandler consoleHandler = new ConsoleHandler();
            consoleHandler.setFormatter(Formatter);
            consoleHandler.setLevel(Level.ALL); // We will filter the messages in our own filter
            // Here's where we filter the messages
            consoleHandler.setFilter(new LogFilter(Level.CONFIG));

            // Disable the default handler
            logger.setUseParentHandlers(false);
            logger.addHandler(consoleHandler); // Add our own handler

            Loggers.put(loggerName, logger);

            logger.log(Level.CONFIG, "Logger created: " + loggerName);
        }

        return logger;
    }

    /** Sets the minimum level for a message to be logged */
    public default void setLogFilterLevel(Level level)
    {
        ((LogFilter) getLogger().getHandlers()[0].getFilter()).setLevel(level);
    }

    public default void log(Level level, String message)
    {
        getLogger().log(level, message);
    }

    public default void logInfo(String message)
    {
        log(Level.INFO, message);
    }

    public default void logWarning(String message)
    {
        log(Level.WARNING, message);
    }

    public default void logSevere(String message)
    {
        log(Level.SEVERE, message);
    }

    public default void logException(Exception e)
    {
        String msg = e.getMessage();

        StackTraceElement[] stackTrace = e.getStackTrace();
        for (int i = 0; i < Math.min(STACK_TRACE_LENGTH, stackTrace.length); i++)
            msg += "\n\t" + stackTrace[i].toString();

        log(Level.SEVERE, msg);
    }

    public default void logConfig(String message)
    {
        log(Level.CONFIG, message);
    }

    public default void logFine(String message)
    {
        log(Level.FINE, message);
    }

    public default void logFiner(String message)
    {
        log(Level.FINER, message);
    }

    public default void logFinest(String message)
    {
        log(Level.FINEST, message);
    }

}
