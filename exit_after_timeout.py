import subprocess
import time
import sys

# Maximum execution time in seconds
timeout = 10

# Your main script command
main_script_command = ["python", "src/orangepi/ProductionFiles/FinalPiProgram.py -test"]

start_time = time.time()

try:
    # Run the main script in a subprocess
    subprocess.run(main_script_command, check=True)
except subprocess.CalledProcessError as e:
    print(f"Error: {e}")
    sys.exit(1)
except KeyboardInterrupt:
    print("Process interrupted by user")
    sys.exit(1)
except Exception as e:
    print(f"Unexpected error: {e}")
    sys.exit(1)
finally:
    elapsed_time = time.time() - start_time
    print(f"Elapsed time: {elapsed_time} seconds")

    # If the elapsed time exceeds the timeout, exit with an error code
    if elapsed_time > timeout:
        sys.exit(1)
