import os
from openai import OpenAI
import json


class Parser:
    def __init__(self, api_key):
        self.client = OpenAI(api_key=api_key)

    def parse_sentence(self, sentence):
        prompt = f"""
        Extract the object and destination from the given command.
        Return a JSON object with "object" and "destination" fields.

        We have 4 possible destination, top-left, top-right, bottom-left, bottom-right.
        the number of top-left is 0, top-right is 1, bottom-left is 2, bottom-right is 3.

        Example:
        Input: "Pick the apple to the top-left corner."
        Output: {{"object": "apple.", "destination": 0}}

        Now, process this sentence:
        Input: "{sentence}"
        Output:
        """

        response = self.client.chat.completions.create(
            model="gpt-4o-mini",  # You can use "gpt-3.5-turbo" for lower cost
            messages=[
                {
                    "role": "system",
                    "content": "You extract objects and destinations from sentences.",
                },
                {"role": "user", "content": prompt},
            ],
        )

        result = response.choices[0].message.content
        try:
            return json.loads(result)
        except json.JSONDecodeError:
            return {"error": "Failed to parse response"}


if __name__ == "__main__":
    from dotenv import load_dotenv

    load_dotenv()

    api_key = os.getenv("OPENAI_API_KEY")
    parser = Parser(api_key)
    sentence = "Pick the banana to the top-right corner."
    parsed_result = parser.parse_sentence(sentence)
    print(parsed_result)
