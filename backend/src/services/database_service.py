import asyncpg
import os
import json
from typing import Optional, List, Literal
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

    async def save_message(
        self,
        session_id: UUID,
        role: Literal['user', 'assistant'],
        content: str,
        citations: Optional[List[Citation]] = None,
        query_type: Optional[Literal['full_text', 'text_selection']] = None
    ) -> SessionMessage:
        """
        Saves a chat message to the database and updates the session's activity.
        """
        if not self.connection_pool:
            await self.connect()

        async with self.connection_pool.acquire() as connection:
            async with connection.transaction():
                citations_json = [c.model_dump() for c in citations] if citations else None
                insert_query = """
                    INSERT INTO session_messages (session_id, role, content, citations, query_type, created_at)
                    VALUES ($1, $2, $3, $4::jsonb, $5, $6)
                    RETURNING id, session_id, role, content, citations, query_type, created_at;
                """
                now = datetime.utcnow()
                message_record = await connection.fetchrow(
                    insert_query, session_id, role, content, citations_json, query_type, now
                )

                update_session_query = """
                    UPDATE query_sessions
                    SET message_count = message_count + 1, last_activity = $1
                    WHERE id = $2;
                """
                await connection.execute(update_session_query, now, session_id)

                if message_record:
                    return SessionMessage(
                        id=message_record['id'],
                        session_id=message_record['session_id'],
                        role=message_record['role'],
                        content=message_record['content'],
                        citations=[Citation(**c) for c in message_record['citations']] if message_record['citations'] else None,
                        query_type=message_record['query_type'],
                        created_at=message_record['created_at']
                    )
                else:
                    raise Exception("Failed to save message.")

    async def get_session_messages(self, session_id: UUID) -> List[SessionMessage]:
        """
        Retrieves all messages for a given session, ordered by creation time.
        """
        if not self.connection_pool:
            await self.connect()

        query = """
            SELECT id, session_id, role, content, citations, query_type, created_at
            FROM session_messages
            WHERE session_id = $1
            ORDER BY created_at;
        """
        records = await self.fetch(query, session_id)

        messages = []
        for record in records:
            messages.append(SessionMessage(
                id=record['id'],
                session_id=record['session_id'],
                role=record['role'],
                content=record['content'],
                citations=[Citation(**c) for c in record['citations']] if record['citations'] else None,
                query_type=record['query_type'],
                created_at=record['created_at']
            ))
        return messages


# Global instance to be used throughout the application
db_service = DatabaseService()
