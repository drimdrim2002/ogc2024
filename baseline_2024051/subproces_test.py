import subprocess
import json


def main():
    jar_path = 'OgcHeuristics.jar'

    # Numbers to be summed (replace with your desired values)
    num1 = 5
    num2 = 10

    param = {}
    param['name'] = 'hyeokjun.son'

    # Build the command to execute the JAR with arguments
    command = ["java", "-jar", jar_path, str(param)]  # Convert numbers to strings

    try:

        # Execute the JAR and capture the output (may include errors)
        result = subprocess.run(command, capture_output=True, text=True, check=True)

        # Extract the sum result from the output (assuming the JAR returns it on a single line)
        sum_result = result.stdout.strip()

        print(sum_result)

        # loads = json.loads(sum_result)
        # print(loads)
    except subprocess.CalledProcessError as error:
        print(f"Error executing JAR: {error}")


if __name__ == "__main__":
    main()
