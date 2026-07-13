from jinja2 import (
    Environment,
    FileSystemLoader,
    StrictUndefined
)

env = Environment(
    loader=FileSystemLoader(
        '/home/nilton/Desktop/Ros2/mobi_robots/src/arms/arm_package_builder/templates/'),
    undefined=StrictUndefined
)

tempmlate = env.get_template('arm/description/CMakeLists.txt.j2')

print(tempmlate.render({'arm': 'gen3'}))
