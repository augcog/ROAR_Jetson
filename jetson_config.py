from pydantic import BaseModel, Field


class JetsonConfig(BaseModel):
    jetson_sudo_password_file_path: str = Field(default="configurations/jetson_sudo_setup.json")
    pygame_display_width: int = Field(default=800)
    pygame_display_height: int = Field(default=600)
    initiate_pygame: bool = Field(default=True)

