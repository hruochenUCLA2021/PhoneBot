pip install huggingface_hub


hf download rhasspy/piper-voices \
  --include "en/**" \
  --local-dir voices

hf download rhasspy/piper-voices \
  --include "zh/**" \
  --local-dir voices


  

Yes — the error is because you used the wrong repo type.

rhasspy/piper-voices is a model repo, not a dataset repo, and the page shows it as ONNX/model content with folders like en and zh.

Also, hf download works best for files or filtered repo downloads, not by treating en as a standalone downloadable file. Hugging Face’s docs recommend downloading a whole repo or filtering with patterns like allow_patterns / CLI --include.

Use this instead:

hf download rhasspy/piper-voices \
  --include "en/**" \
  --local-dir voices

hf download rhasspy/piper-voices \
  --include "zh/**" \
  --local-dir voices

Or in one command:

hf download rhasspy/piper-voices \
  --include "en/**" \
  --include "zh/**" \
  --local-dir voices

A few notes:

Do not use --repo-type dataset here.
huggingface-cli download is deprecated; hf download is the current command. Your warning is expected. The Hugging Face CLI docs show hf download as the modern command.
You can preview size first with:
hf download rhasspy/piper-voices \
  --include "en/**" \
  --include "zh/**" \
  --dry-run

The docs show --dry-run for checking what would be downloaded before actually downloading.

If you want the smallest practical setup, it is still better to download just 1–2 specific voices instead of all English and Chinese voices, because the repo is about 10.1 GB overall and contains many language folders and voices.






