# chatbot_backend/database.py
import os
from typing import Generator
from sqlmodel import create_engine, SQLModel, Session
from dotenv import load_dotenv

# Load environment variables from .env file
# Ensure you have a .env file in the chatbot_backend directory with:
# DATABASE_URL="postgresql://user:password@host:port/dbname"
load_dotenv()

DATABASE_URL = os.getenv("DATABASE_URL")

if not DATABASE_URL:
    raise ValueError("DATABASE_URL environment variable is not set. Please create a .env file.")

# The 'connect_args' are recommended for serverless Postgres providers like Neon
# to prevent issues with connection pooling.
engine = create_engine(DATABASE_URL, connect_args={"sslmode": "require"}, pool_recycle=300)

def create_db_and_tables():
    """
    Initializes the database and creates all tables defined by SQLModel metadata.
    This function should be called once on application startup.
    """
    print("Initializing database and creating tables...")
    SQLModel.metadata.create_all(engine)
    print("Database and tables initialized successfully.")

def get_session() -> Generator[Session, None, None]:
    """
    Dependency to get a database session for a single request.
    Yields a session and ensures it's closed after the request is handled.
    """
    with Session(engine) as session:
        yield session

