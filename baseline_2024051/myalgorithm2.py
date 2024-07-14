import base64
import os
import subprocess


def algorithm(K, all_orders, all_riders, dist_mat, timelimit=60):

    jar_path = get_jar_file_name()

    original_data = {}
    original_data['orders'] = all_orders
    original_data['riders'] = all_riders
    original_data['dist'] = dist_mat
    original_data_str = str(original_data)

    original_data_byte = original_data_str.encode('utf-8')
    print(f'original_data_byte: {original_data_byte}')
    encodes_data_base64 = base64.b64encode(original_data_byte)
    print(f'encodes_data_base64: {encodes_data_base64}')
    encoded_data_str = encodes_data_base64.decode('utf-8')
    print(f'encoded_data_str: {encoded_data_str}')

    # param_length = len(original_data_str)
    # compress_data = zlib.compress(original_data_str.encode(encoding='utf-8'))
    # print(f'compress_data: {compress_data}')

    # Build the command to execute the JAR with arguments
    command = ["java", "-jar", jar_path, encoded_data_str]  # Convert numbers to strings

    try:

        # Execute the JAR and capture the output (may include errors)
        result = subprocess.run(command, capture_output=True, encoding='utf-8')
        print(f'result error: {result.stderr}')

        # Extract the sum result from the output (assuming the JAR returns it on a single line)
        sum_result = result.stdout.strip()

        print(f'sum_result: {sum_result}')

        # loads = json.loads(sum_result)
        # print(loads)
    except subprocess.CalledProcessError as error:
        print(f"Error executing JAR: {error}")


def get_jar_file_name():
    jar_path = os.path.realpath(__file__)
    jar_path = jar_path[0: jar_path.rfind("\\")]
    jar_path += "\OgcHeuristics.jar"
    return jar_path


