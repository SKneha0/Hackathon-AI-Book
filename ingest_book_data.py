# ingest_book_data.py
import os
import requests
import re
from typing import List, Dict

# Configuration
FASTAPI_INGEST_URL = "http://localhost:8000/ingest_data" # Ensure your FastAPI backend is running
DOCS_DIR = "docs" # Directory containing your markdown book chapters

def get_markdown_files(directory: str) -> List[str]:
    """
    Recursively finds all markdown files in the given directory.
    """
    markdown_files = []
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(".md") or file.endswith(".mdx"):
                markdown_files.append(os.path.join(root, file))
    return markdown_files

def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
    """
    Splits text into chunks of a given size with overlap.
    Aims to split at paragraph breaks or sentence endings if possible.
    """
    # Remove YAML front matter
    text = re.sub(r'---.*?---', '', text, flags=re.DOTALL)
    
    # Simple chunking for now, can be improved with more sophisticated methods
    chunks = []
    current_pos = 0
    while current_pos < len(text):
        end_pos = min(current_pos + chunk_size, len(text))
        chunk = text[current_pos:end_pos]
        chunks.append(chunk.strip())
        current_pos += chunk_size - overlap
        if current_pos >= len(text) - overlap: # Avoid tiny last chunks
            break
    return chunks

def ingest_data_to_backend(chunks: List[str]):
    """
    Sends text chunks to the FastAPI backend's /ingest_data endpoint.
    """
    try:
        response = requests.post(FASTAPI_INGEST_URL, json={"chunks": chunks})
        response.raise_for_status()  # Raise an exception for HTTP errors
        print(f"Ingestion successful: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"Error during ingestion: {e}")
        print(f"Response content: {e.response.text if e.response else 'N/A'}")

def main():
    print(f"Starting data ingestion from '{DOCS_DIR}'...")
    all_chunks = []
    markdown_files = get_markdown_files(DOCS_DIR)

    if not markdown_files:
        print(f"No markdown files found in '{DOCS_DIR}'.")
        return

    for md_file in markdown_files:
        print(f"Processing file: {md_file}")
        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()
        
        file_chunks = chunk_text(content)
        all_chunks.extend(file_chunks)
        print(f"  Generated {len(file_chunks)} chunks from {md_file}")

    if all_chunks:
        print(f"\nTotal chunks to ingest: {len(all_chunks)}")
        # Ingest in batches to avoid extremely large requests if there are many chunks
        batch_size = 50
        for i in range(0, len(all_chunks), batch_size):
            batch_chunks = all_chunks[i:i + batch_size]
            print(f"Ingesting batch {i // batch_size + 1} of {len(all_chunks) // batch_size + 1} ({len(batch_chunks)} chunks)...")
            ingest_data_to_backend(batch_chunks)
    else:
        print("No chunks generated to ingest.")

if __name__ == "__main__":
    main()
