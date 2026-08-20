"""Show the compact 810x510 vehicle with its resized cargo module."""

import os

os.environ["VEHICLE_PAYLOAD"] = "cargo"

from vehicle_shell_integration import main


if __name__ == "__main__":
    main()
