from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # ---- Launch args ----
    model_path = LaunchConfiguration("model_path")
    audio_topic = LaunchConfiguration("audio_topic")

    # audio_capture params
    sample_rate = LaunchConfiguration("sample_rate")
    channels = LaunchConfiguration("channels")
    sample_format = LaunchConfiguration("sample_format")
    audio_format = LaunchConfiguration("audio_format")
    device = LaunchConfiguration("device")

    publish_partial = LaunchConfiguration("publish_partial")

    # TTS params
    tts_engine = LaunchConfiguration("tts_engine")
    tts_voice = LaunchConfiguration("tts_voice")
    tts_speed = LaunchConfiguration("tts_speed")
    tts_pitch = LaunchConfiguration("tts_pitch")
    tts_volume = LaunchConfiguration("tts_volume")
    tts_interrupt = LaunchConfiguration("tts_interrupt")

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value="/home/oscar/.cache/vosk/vosk-model-small-en-us-0.15",
            description="Path to Vosk model directory"
        ),
        DeclareLaunchArgument(
            "audio_topic",
            default_value="/audio",
            description="Audio topic to subscribe/publish"
        ),
        DeclareLaunchArgument(
            "sample_rate",
            default_value="16000",
            description="Audio capture sample rate"
        ),
        DeclareLaunchArgument(
            "channels",
            default_value="1",
            description="Audio capture channels"
        ),
        DeclareLaunchArgument(
            "sample_format",
            default_value="S16LE",
            description="Audio sample format for capture (e.g., S16LE)"
        ),
        DeclareLaunchArgument(
            "audio_format",
            default_value="wave",
            description='audio_capture output format: MUST be "wave" or "mp3" in this build'
        ),
        DeclareLaunchArgument(
            "device",
            default_value="",
            description="Optional audio device string/index (leave empty for default)"
        ),
        DeclareLaunchArgument(
            "publish_partial",
            default_value="true",
            description="Publish partial recognition results"
        ),

        # ---- TTS args (espeak backend for now) ----
        DeclareLaunchArgument(
            "tts_engine",
            default_value="",
            description='TTS engine binary. Empty = auto ("espeak-ng" then "espeak")'
        ),
        DeclareLaunchArgument(
            "tts_voice",
            default_value="",
            description='TTS voice (e.g., "en-us", "en", "es-la"). Empty = default'
        ),
        DeclareLaunchArgument(
            "tts_speed",
            default_value="160",
            description="TTS speed (words per minute)"
        ),
        DeclareLaunchArgument(
            "tts_pitch",
            default_value="50",
            description="TTS pitch (0-99)"
        ),
        DeclareLaunchArgument(
            "tts_volume",
            default_value="100",
            description="TTS volume/amplitude (commonly 0-200)"
        ),
        DeclareLaunchArgument(
            "tts_interrupt",
            default_value="true",
            description="If true, new TTS request stops current speech"
        ),

        # ---- Audio Capture ----
        Node(
            package="audio_capture",
            executable="audio_capture_node",
            name="audio_capture_node",
            output="screen",
            parameters=[{
                "sample_rate": sample_rate,
                "channels": channels,
                "sample_format": sample_format,
                "format": audio_format,
                "device": device,
            }]
        ),

        # ---- Vosk STT ----
        Node(
            package="robotino_audio",
            executable="vosk_node",
            name="vosk_stt_node",
            output="screen",
            parameters=[{
                "audio_topic": audio_topic,
                "model_path": model_path,
                "sample_rate": 16000,  # match capture
                "publish_partial": publish_partial,
            }]
        ),

        # ---- TTS (eSpeak for now) ----
        Node(
            package="robotino_tts",
            executable="espeak_tts_node",
            name="tts_node",
            output="screen",
            parameters=[{
                "engine": tts_engine,
                "voice": tts_voice,
                "speed": tts_speed,
                "pitch": tts_pitch,
                "volume": tts_volume,
                "interrupt": tts_interrupt,
            }]
        ),
    ])
