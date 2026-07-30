# Raw recordings

Raw `*.svo2` and BODY_38 `*.jsonl` recordings are not committed:

- SVO2 files are hundreds of MB or several GB.
- Human video/skeleton recordings may contain personal data.
- GitHub blocks ordinary files larger than 100 MB.

Keep the originals in encrypted project storage. The small derived 23-DOF
`datasets/*.npz`, tracking clips and quality manifests in this repository are
the reproducible simulation inputs.
