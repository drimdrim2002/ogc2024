import subprocess
import zlib
import os

def algorithm(K, all_orders, all_riders, dist_mat, timelimit=60):

    jar_path = os.path.realpath(__file__)
    jar_path = jar_path[0: jar_path.rfind("\\")]
    print(jar_path)
    jar_path += "\OgcHeuristics.jar"
    print(jar_path)

    param = {}
    param['orders'] = all_orders
    param['riders'] = all_riders
    param['dist'] = dist_mat

    param_str = str(param)
    compress_data = zlib.compress(param_str.encode(encoding='utf-8'))

    print(f'compress_data: {compress_data}')
    # Build the command to execute the JAR with arguments
    command = ["java", "-jar", jar_path, param_str]  # Convert numbers to strings

    try:

        # Execute the JAR and capture the output (may include errors)
        result = subprocess.run(command, capture_output=True, encoding='utf8')
        print(f'result error: {result.stderr}')

        # Extract the sum result from the output (assuming the JAR returns it on a single line)
        sum_result = result.stdout.strip()

        print(f'sum_result: {sum_result}')

        # loads = json.loads(sum_result)
        # print(loads)
    except subprocess.CalledProcessError as error:
        print(f"Error executing JAR: {error}")


