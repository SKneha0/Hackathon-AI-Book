# chatbot_backend/main.py
import os
from contextlib import asynccontextmanager
from typing import List, Dict
from datetime import datetime
from fastapi import FastAPI, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from sqlmodel import Field, Session, SQLModel
from openai import OpenAI
from dotenv import load_dotenv

from database import create_db_and_tables, get_session, engine
from vector_store import vector_store_manager

# Load environment variables
# Ensure you have a .env file in the chatbot_backend directory with:
# OPENAI_API_KEY="your_openai_api_key"
load_dotenv()

OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

if not OPENAI_API_KEY:
    raise ValueError("OPENAI_API_KEY environment variable is not set. Please create a .env file.")

# Initialize OpenAI client
openai_client = OpenAI(api_key=OPENAI_API_KEY)

# --- SQLModel for Chat History ---
class ChatHistory(SQLModel, table=True):
    id: int = Field(default=None, primary_key=True)
    question: str
    answer: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)

# --- FastAPI App Lifecycle ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Handles startup and shutdown events for the FastAPI application.
    On startup, it ensures the database tables are created.
    """
    print("FastAPI app starting up...")
    create_db_and_tables()
    # You might want to pre-populate the vector store here if it's empty
    # This example assumes the vector store is populated externally or on first use.
    print("FastAPI app startup complete.")
    yield
    print("FastAPI app shutting down.")

app = FastAPI(lifespan=lifespan)

# --- CORS Middleware ---
# Configure CORS to allow requests from your Docusaurus frontend
# Replace "http://localhost:3000" with your Docusaurus URL in production
origins = [
    "http://localhost:3000",  # Default Docusaurus development server
    # Add your deployed Docusaurus URL here when you deploy
    # "https://your-deployed-docusaurus-url.com",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Endpoints ---
@app.get("/")
async def read_root():
    return {"message": "Welcome to the RAG Chatbot Backend!"}

@app.post("/query")
async def query_chatbot(
    question_data: Dict[str, str],
    session: Session = Depends(get_session)
):
    question = question_data.get("question")
    if not question:
        raise HTTPException(status_code=400, detail="Question cannot be empty.")

    print(f"Received question: '{question}'")

    # 1. Retrieve relevant context from Qdrant
    relevant_chunks = vector_store_manager.query_vector_store(question, limit=5)
    
    if not relevant_chunks:
        answer = "I could not find relevant information in the book to answer your question."
        print("No relevant chunks found. Answering with fallback.")
    else:
        context = "\n".join(relevant_chunks)
        print(f"Context retrieved: {context[:200]}...") # Print first 200 chars of context

        # 2. Generate answer using OpenAI
        try:
            response = openai_client.chat.completions.create(
                model="gpt-3.5-turbo", # You can use a more advanced model like "gpt-4"
                messages=[
                    {"role": "system", "content": "You are a helpful assistant specialized in 'Physical AI & Humanoid Robotics'. Answer the user's question ONLY based on the provided context. If the answer is not in the context, state that you don't have enough information."},
                    {"role": "user", "content": f"Context: {context}\n\nQuestion: {question}\nAnswer:"}
                ],
                max_tokens=500,
                temperature=0.7,
                stream=False
            )
            answer = response.choices[0].message.content.strip()
            print(f"Generated answer: {answer[:200]}...")
        except Exception as e:
            print(f"Error generating answer with OpenAI: {e}")
            answer = "An error occurred while trying to generate an answer."

    # 3. Store question and answer in Neon Postgres
    try:
        chat_entry = ChatHistory(question=question, answer=answer)
        session.add(chat_entry)
        session.commit()
        session.refresh(chat_entry)
        print(f"Chat history saved to DB: ID {chat_entry.id}")
    except Exception as e:
        print(f"Error saving chat history to database: {e}")
        # Log the error but don't prevent the answer from being sent

    return {"answer": answer}

# --- Utility to ingest data (Optional, for testing/initial setup) ---
@app.post("/ingest_data")
async def ingest_data(data: Dict[str, List[str]]):
    """
    Endpoint to ingest text chunks into the vector store.
    This is for initial population of Qdrant with book content.
    """
    chunks = data.get("chunks")
    if not chunks:
        raise HTTPException(status_code=400, detail="No text chunks provided for ingestion.")
    
    try:
        vector_store_manager.add_text_chunks(chunks)
        return {"status": "success", "message": f"Ingested {len(chunks)} chunks."}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to ingest data: {e}")

# --- Run the application (for development) ---
# To run this: uvicorn main:app --reload --port 8000
# Ensure you are in the chatbot_backend directory