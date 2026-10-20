# Contract — NN01 genome file with layout signature (044)

1. `NN_FORMAT_VERSION` 1 → 2; a new u64 `layout_signature` follows the existing metadata fields.
2. Writers (autoc save, nnextractor output) stamp the signature of the tree that produced the genome's inputs.
3. `nnextractor` and `nn2cpp` refuse a genome whose signature differs from the tree's or whose `topology[0]` ≠ `NN_INPUT_COUNT`,
   naming both signatures; old-version files fail loud (Constitution V).
4. Generated firmware carries `static_assert(kGeneratedLayoutSignature == kNNInputLayoutSignature)` beside the count and
   scale-signature asserts.
