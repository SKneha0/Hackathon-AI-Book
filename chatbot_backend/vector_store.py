# chatbot_backend/vector_store.py
import os
from typing import List, Dict, Any
from qdrant_client import QdrantClient, models
from qdrant_client.http.models import Distance, VectorParams
from sentence_transformers import SentenceTransformer
from dotenv import load_dotenv

# Load environment variables
# Ensure you have a .env file in the chatbot_backend directory with:
# QDRANT_URL="your_qdrant_url"
# QDRANT_API_KEY="your_qdrant_api_key"
load_dotenv()

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

if not QDRANT_URL:
    raise ValueError("QDRANT_URL environment variable is not set.")
if not QDRANT_API_KEY:
    raise ValueError("QDRANT_API_KEY environment variable is not set.")

COLLECTION_NAME = "book_chunks"
# Initialize the embedding model outside of functions to load it once
# You can choose a different model if needed, e.g., 'all-MiniLM-L6-v2' for smaller size
embedding_model = SentenceTransformer("all-MiniLM-L6-v2")

class VectorStoreManager:
    def __init__(self):
        self.client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
        )
        self.vector_size = embedding_model.get_sentence_embedding_dimension()
        self._create_collection_if_not_exists()

    def _create_collection_if_not_exists(self):
        """
        Creates the Qdrant collection if it doesn't already exist.
        """
        try:
            self.client.get_collection(collection_name=COLLECTION_NAME)
            print(f"Collection '{COLLECTION_NAME}' already exists.")
        except Exception:
            print(f"Creating collection '{COLLECTION_NAME}'...")
            self.client.recreate_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=VectorParams(size=self.vector_size, distance=Distance.COSINE),
            )
            print(f"Collection '{COLLECTION_NAME}' created.")

    def add_text_chunks(self, chunks: List[str], metadatas: List[Dict[str, Any]] = None):
        """
        Embeds a list of text chunks and stores them in the Qdrant collection.
        :param chunks: List of text strings to embed and store.
        :param metadatas: Optional list of metadata dictionaries, one for each chunk.
        """
        print(f"Adding {len(chunks)} text chunks to Qdrant...")
        embeddings = embedding_model.encode(chunks, show_progress_bar=True).tolist()

        if metadatas and len(metadatas) != len(chunks):
            raise ValueError("Length of metadatas must match length of chunks if provided.")
        
        points = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            payload = {"text_content": chunk}
            if metadatas and metadatas[i]:
                payload.update(metadatas[i])
            points.append(
                models.PointStruct(
                    id=self.client.count(collection_name=COLLECTION_NAME).count + i + 1, # Simple ID generation
                    vector=embedding,
                    payload=payload
                )
            )

        self.client.upsert(
            collection_name=COLLECTION_NAME,
            points=points,
            wait=True,
        )
        print(f"Successfully added {len(chunks)} text chunks to Qdrant.")

    def query_vector_store(self, question: str, limit: int = 3) -> List[str]:
        """
        Queries the vector store for relevant text chunks based on the question.
        :param question: The question string to query.
        :param limit: The maximum number of relevant chunks to retrieve.
        :return: A list of relevant text chunks.
        """
        print(f"Querying Qdrant for question: '{question}'...")
        query_embedding = embedding_model.encode(question).tolist()

        search_result = self.client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=limit
        )
        
        relevant_texts = [hit.payload["text_content"] for hit in search_result if hit.payload and "text_content" in hit.payload]
        print(f"Retrieved {len(relevant_texts)} relevant chunks.")
        return relevant_texts

# Global instance for easy access
vector_store_manager = VectorStoreManager()
