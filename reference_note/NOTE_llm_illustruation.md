# NOTE: LLM request/response structure (OpenAI Chat Completions)

This note explains the parts in `AI_backend/server.py` that talk to OpenAI:

- **Request**: `server.py` builds `headers` + `body` and sends an HTTPS POST
- **Response**: OpenAI returns JSON, we read from `choices[0].message.content`

Relevant code:

```98:121:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/AI_backend/server.py
def openai_chat(prompt: str, model: str, system_prompt: str) -> str:
    url = "https://api.openai.com/v1/chat/completions"
    headers = {"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"}
    body = {
        "model": model,
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": prompt},
        ],
    }
    r = requests.post(url, headers=headers, data=json.dumps(body), timeout=60)
    data = r.json()
    return data["choices"][0]["message"]["content"].strip()
```

---

## What are “headers” vs “body”?

### Headers

**Headers are metadata about the HTTP request**. In our OpenAI call we use:

- `Authorization: Bearer <OPENAI_API_KEY>`
  - proves to OpenAI “this request is allowed”
- `Content-Type: application/json`
  - tells OpenAI “the request body is JSON text”

Headers are not the prompt itself; they’re “envelope” information.

### Body

**Body is the main payload** (the JSON content) that contains:

- which model to use
- what messages to answer

We build a Python dictionary `body = {...}` and send it as JSON.

---

## What is `model`?

In the request body:

- `"model": model`

This selects which OpenAI model to run (example: `gpt-4o-mini`).

Different models have different:

- speed
- cost
- quality
- context window

---

## What is `messages`?

The Chat Completions API is “chat style”.

Instead of sending one big string, you send an ordered list of messages:

- system message: sets behavior (“be concise”, “you are a robotics assistant”, rules)
- user message: the user’s request (“transcript” or typed text)

The model replies as an “assistant message”.

In our backend we send only two messages:

- `system` (instructions)
- `user` (the prompt text)

---

## What are `role` and `content`?

Each item in `messages` is like:

```json
{ "role": "user", "content": "Hello" }
```

- `role` is who said it:
  - `system` = “rules / style / constraints”
  - `user` = “question/request”
  - `assistant` = “the model’s previous reply” (not sent in our minimal version)
- `content` is the actual text.

---

## What is `choices` in the response?

OpenAI may return multiple candidate answers (especially if you ask for `n > 1`).
Those candidates are returned in:

- `data["choices"]`

Each `choice` includes a generated message.

In our code we just take the **first** one:

- `choices[0]`

---

## What is `data["choices"][0]["message"]["content"]`?

This is “the model’s answer text”.

Breakdown:

- `data` = JSON response as a Python dict
- `choices` = list of candidate completions
- `[0]` = first candidate
- `message` = the assistant message object
- `content` = the text content of the assistant message

That’s what we return to Android as `reply`.

---

## A concrete mini example

If you send:

```json
{
  "model": "gpt-4o-mini",
  "messages": [
    {"role": "system", "content": "Be concise."},
    {"role": "user", "content": "Say hello"}
  ]
}
```

You might get back something like:

```json
{
  "choices": [
    {
      "message": {
        "role": "assistant",
        "content": "Hello!"
      }
    }
  ]
}
```

Our code picks `"Hello!"`.

