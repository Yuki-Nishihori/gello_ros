from setuptools import setup

package_name = 'gello_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yusaku Nakajima',
    maintainer_email='yusaku_nakajima@ap.eng.osaka-u.ac.jp',
    description='The gello_ros package for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 例: 実行可能スクリプトを登録（ファイル名:main関数）
            # 'gello_node = gello_ros.gello_node:main',
        ],
    },
)
