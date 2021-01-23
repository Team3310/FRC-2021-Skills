package org.frcteam2910.common.src.main.java.org.frcteam2910.common;

import org.frcteam2910.common.Logger;

public abstract class CrashTrackingRunnable implements Runnable {
	@Override
	public final void run() {
		try {
			runCrashTracked();
		} catch (Throwable t) {
			Logger.getAnonymousLogger().error(t);
			throw t;
		}
	}

	public abstract void runCrashTracked();
}
