from setuptools import find_packages, setup
import os

package_name = "chess_voice"


def collect(directory):
    files = []
    for root, _, filenames in os.walk(directory):
        for filename in filenames:
            full = os.path.join(root, filename)
            install_dir = os.path.join("share", package_name, root)
            files.append((install_dir, [full]))
    return files


data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]
for directory in ["config", "launch"]:
    if os.path.isdir(directory):
        data_files.extend(collect(directory))


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=[
        "setuptools",
        # Runtime deps installed via pip, not rosdep:
        # transformers, torch, sounddevice, numpy, langchain, langchain-huggingface
    ],
    zip_safe=True,
    maintainer="Pablo Revuelto",
    maintainer_email="bloparev12@gmail.com",
    description="ASR + LangChain voice command pipeline.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "audio_capture = chess_voice.audio_capture:main",
            "whisper_asr   = chess_voice.whisper_asr_node:main",
            "voice_parser  = chess_voice.voice_parser_node:main",
        ],
    },
)
