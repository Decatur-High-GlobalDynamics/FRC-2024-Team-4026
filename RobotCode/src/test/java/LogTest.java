import frc.lib.core.ILogSource;

import java.util.logging.Level;
import org.junit.jupiter.api.Test;

class LogTest implements ILogSource
{
    // Uncommnet @Test to run the test on build
    // @Test
    void testLog()
    {
        System.out.println("Testing log functions");

        logConfig("This is a config message");
        // logInfo("This is an info message");
        // logWarning("This is a warning message");
        // logSevere("This is a severe message");
        // logException(new Exception("This is an exception message"));
        logConfig("This is a config message");

        logFine("This is a fine message that should not be printed");
        setLogFilterLevel(Level.FINE);
        logFine("This is another fine message");
        logFiner("This is a finer message and should not be printed");

        setLogFilterLevel(Level.FINEST);
        logFinest("This is the finest message");

        System.out.println("Log functions tested");

        assert (true);
    }
}
