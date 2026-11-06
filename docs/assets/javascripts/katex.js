/* Renders the formulas of pymdownx.arithmatex (generic mode) with KaTeX, also after
   instant navigation. SPDX-License-Identifier: Apache-2.0 */
document$.subscribe(({ body }) => {
  renderMathInElement(body, {
    delimiters: [
      { left: "$$", right: "$$", display: true },
      { left: "$", right: "$", display: false },
      { left: "\\(", right: "\\)", display: false },
      { left: "\\[", right: "\\]", display: true }
    ],
    throwOnError: false
  });
});
