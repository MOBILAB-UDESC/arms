import os
import re


def replace_string(file_path, old_strings, new_strings):
    """
    Read a template file, perform replacements, and save it with the same extension.

    Args:
        file_path (str): Path to the template file.
        old_strings (list of str): List of strings to be replaced.
        new_strings (list of str): List of strings to replace with.

    Returns:
        str: Path to the newly created file.
    """
    if len(old_strings) != len(new_strings):
        raise ValueError("old_strings and new_strings must be of the same length.")

    with open(file_path, 'r') as srdf_file:
        srdf_content = srdf_file.read()

    file_name = os.path.basename(file_path)

    # Get the file extension of file_name.
    # If file_name contains '_template' before the extension, this part is removed.
    ext = re.sub(r'.*\.', '', file_name)
    base = re.sub(r'_template\.[^.]+$', '', file_name)

    for old_string, new_string in zip(old_strings, new_strings):
        srdf_content = srdf_content.replace(old_string, new_string)

    new_path = os.path.join(os.path.dirname(file_path), f'{base}.{ext}')

    with open(new_path, 'w') as f:
        f.write(srdf_content)

    # print(f'New path: {new_path}')

    return new_path


def main():
    from ament_index_python.packages import get_package_share_directory

    test_file = os.path.join(get_package_share_directory('gen3_lite_moveit_config'),
                             'config', 'gen3_lite_kinova_2f_lite', 'gen3_lite_template.srdf')

    print(f'Test path: {test_file}')
    print(f'New path: {replace_string(test_file, '<robot_prefix>', 'kinova_')}')


if __name__ == "__main__":
    main()
