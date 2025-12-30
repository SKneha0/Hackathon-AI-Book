---
id: 1
title: "Fix Homepage Crash on Missing Blog Data"
stage: "green"
date: "2025-12-30"
surface: "agent"
model: "gemini-pro"
feature: "homepage"
branch: "main"
user: "user"
command: "I have a Docusaurus project where the Home page is crashing with this error: \"This page crashed. Cannot destructure property 'blogPosts' of '(0 , _docusaurus_useGlobalData__WEBPACK_IMPORTED_MODULE_4__.usePluginData)(...)' as it is undefined.\" Please do the following: 1. Analyze all existing files in the Gemini project folder, including src/pages, docusaurus.config.js, and plugins configuration. 2. Identify why usePluginData() is returning undefined for 'blogPosts' on the Home page. 3. Fix the Home page code so that it safely accesses the blogPosts data without creating any new folders or changing the project structure. 4. Ensure that the fix handles empty or missing blog data gracefully (so the page does not crash). 5. Only modify existing files as needed; do not add any new files or folders. 6. Provide the corrected code snippets for the affected files and explain the changes briefly. Do not generate any new project or folder structure. Work strictly within the current project."
labels: ["bugfix", "docusaurus", "homepage"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
- src/pages/index.js
tests_yaml: |
null
---

### PROMPT

I have a Docusaurus project where the Home page is crashing with this error:

"This page crashed. Cannot destructure property 'blogPosts' of '(0 , _docusaurus_useGlobalData__WEBPACK_IMPORTED_MODULE_4__.usePluginData)(...)' as it is undefined."

Please do the following:

1. Analyze all existing files in the Gemini project folder, including src/pages, docusaurus.config.js, and plugins configuration.
2. Identify why usePluginData() is returning undefined for 'blogPosts' on the Home page.
3. Fix the Home page code so that it safely accesses the blogPosts data without creating any new folders or changing the project structure.
4. Ensure that the fix handles empty or missing blog data gracefully (so the page does not crash).
5. Only modify existing files as needed; do not add any new files or folders.
6. Provide the corrected code snippets for the affected files and explain the changes briefly.

Do not generate any new project or folder structure. Work strictly within the current project.

### RESPONSE

The `src/pages/index.js` file was modified. The `LatestBlogPosts` function was updated to prevent a crash if `blogPosts` data is unavailable. Previously, the code directly destructured `blogPosts` from `usePluginData('docusaurus-plugin-content-blog')`, which caused a crash if the plugin data was `undefined`. The corrected code now safely accesses the blog data. It first assigns the plugin data to a variable and then checks if it exists. If it doesn't, it defaults `blogPosts` to an empty array. This prevents the `.slice` method from being called on an `undefined` value, thus fixing the crash.

```
