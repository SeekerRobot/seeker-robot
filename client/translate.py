from transcribe import text, ingest

from enum import Enum
from pydantic import BaseModel, Field, ValidationError
import json

from google.genai import types

import time

class Input(BaseModel):
    user_command: str
    
class RobotAction(str, Enum):
    FORWARD = 'move forward'
    BACKWARD = 'move backward'
    LEFT = 'move left'
    RIGHT = 'move right'
    SPIN = 'spin'
    DANCE = 'dance'
    FIND = 'find the object'
    NONE = 'NONE OF THE ABOVE'
    
class Output(BaseModel):
    translated_command: RobotAction
    
instruction: str = 'You are a robot named Hatsune. You are given a command by \
    a user that may represent a task from a predefined set of tasks. \
    Unfortunately, the given command will not always be exactly any task. Therefore, \
    you must guess which task the user intented to command. If you do not believe that \
    the command accurately represents any task, opt not to pick any task. The \
    predefined set of tasks is: [\'move forward\', \'move backward\', \'move \
    left\', \'move right\', \'spin\', \'dance\', \'find the object\', \'dance\'\
    \'NONE OF THE ABOVE\']'
    
    
client = genai.Client()

command: list[str] = []
translated_command = ''

if 'hey hatsune' in text.lower():
    while 'over' not in text:
        if ingest == True:
            ingest = False

            command.append(text)
            
        else:
            time.sleep(0.5)


    command.append(text)
    
    injected_prompt: str = f"""
    <user_command>
    {' '.join(command)}
    </user_command>
    """
    
    try:
        response = client.models.generate_content(
            model = 'gemini-2.5-flash',
            contents = injected_prompt,
            config = types.GenerateContentConfig(
                system_instruction = instruction,
                response_mime_type = 'application/json',
                response_schema = Output
            )
        )
        
        full_response_obj = Output(**json.loads(response.text))
    
        translated_command = full_response_obj.translated_command
        
        
        
    except Exception as e:
        print(f'Error: {e}')
            
    command = []
    
    