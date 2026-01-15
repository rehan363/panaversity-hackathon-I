import asyncpg
import os
from typing import Optional
from uuid import UUID, uuid4
from datetime import datetime

from models.session import QuerySession, SessionMessage, Citation # Import QuerySession

class DatabaseService:
    """
    Service for managing database connections and operations using asyncpg.
    Connects to a Neon Postgres database using the NEON_DATABASE_URL environment variable.
    """
    _instance: Optional['DatabaseService'] = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(DatabaseService, cls).__new__(cls)
            cls._instance.connection_pool: Optional[asyncpg.Pool] = None
        return cls._instance

    async def connect(self):
        """
        Establishes a connection pool to the Neon Postgres database.
        """
        if self.connection_pool:
            return

        db_url = os.getenv("NEON_DATABASE_URL")
        if not db_url:
            raise ValueError("NEON_DATABASE_URL environment variable not set.")

        try:
            self.connection_pool = await asyncpg.create_pool(db_url)
            print("Successfully connected to Neon Postgres database.")
        except Exception as e:
            print(f"Failed to connect to Neon Postgres database: {e}")
            raise

    async def disconnect(self):
        """
        Closes the database connection pool.
        """
        if self.connection_pool:
            print("Disconnecting from Neon Postgres database.")
            await self.connection_pool.close()
            self.connection_pool = None

    async def execute(self, query: str, *args):
        """
        Executes a database query without returning results (e.g., INSERT, UPDATE, DELETE).
        """
        if not self.connection_pool:
            await self.connect() # Ensure connection is established

        async with self.connection_pool.acquire() as connection:
            await connection.execute(query, *args)

    async def fetch(self, query: str, *args):
        """
        Executes a database query and returns all results (e.g., SELECT).
        """
        if not self.connection_pool:
            await self.connect() # Ensure connection is established

        async with self.connection_pool.acquire() as connection:
            return await connection.fetch(query, *args)

    async def fetchrow(self, query: str, *args):
        """
        Executes a database query and returns a single row.
        """
        if not self.connection_pool:
            await self.connect() # Ensure connection is established

        async with self.connection_pool.acquire() as connection:
            return await connection.fetchrow(query, *args)

    async def create_session(self) -> QuerySession:
        """
        Creates a new anonymous chat session in the database.
        Returns the created QuerySession object.
        """
        session_token = str(uuid4()) # Generate a unique token
        query = """
            INSERT INTO query_sessions (session_token, created_at, last_activity, message_count)
            VALUES ($1, $2, $3, $4)
            RETURNING id, session_token, user_id, created_at, last_activity, message_count;
        """
        # Use datetime.utcnow() for timezone-naive timestamps in UTC
        now = datetime.utcnow()
        record = await self.fetchrow(query, session_token, now, now, 0)

        if record:
            return QuerySession(
                id=record['id'],
                session_token=record['session_token'],
                user_id=record['user_id'],
                created_at=record['created_at'],
                last_activity=record['last_activity'],
                message_count=record['message_count']
            )
        else:
            raise Exception("Failed to create new session.")

# Global instance to be used throughout the application
db_service = DatabaseService()
