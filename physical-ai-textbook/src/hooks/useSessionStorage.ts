import { useState, useEffect, useCallback } from 'react';
import { ChatMessage, Citation } from '../components/RAGChatbot/types';

const API_BASE_URL = '/api/chat';

/**
 * Custom hook for persisting state to sessionStorage, with an added
 * feature to fetch initial history from a backend API.
 *
 * @param key The key to use in sessionStorage.
 * @param initialValue The initial value to use if no value is found
 in sessionStorage or API.
 * @returns A stateful value, and a function to update it.
 */
function useSessionStorage<T>(key: string, initialValue: T): [T, (
value: T | ((val: T) => T)) => void] {
  // State to store our value
  // Pass initial state function to useState so logic is only execut
ed once
  const [storedValue, setStoredValue] = useState<T>(initialValue);

  const readValue = useCallback((): T => {
    if (typeof window === 'undefined') {
      return initialValue;
    }
    try {
      const item = window.sessionStorage.getItem(key);
      return item ? (JSON.parse(item) as T) : initialValue;
    } catch (error) {
      console.warn(`Error reading sessionStorage key “${key}”:`, er
ror);
      return initialValue;
    }
  }, [initialValue, key]);

  // Function to fetch history from backend
  const fetchHistoryFromBackend = useCallback(async () => {
    if (typeof window === 'undefined') {
      return;
    }

    const sessionId = sessionStorage.getItem('sessionId');
    if (!sessionId) {
      console.log('No session ID found, skipping backend history fe
tch.');
      setStoredValue(readValue()); // Fallback to local storage if n
o session
      return;
    }

    try {
      const response = await fetch(`${API_BASE_URL}/history/${sess
ionId}`);
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data: ChatMessage[] = await response.json();

      // Ensure timestamps are Date objects
      const messagesWithDates = data.map(msg => ({
        ...msg,
        timestamp: new Date(msg.timestamp),
      }));

      // Update state with fetched history, ensuring it's of type T.
      // This assumes T can be assigned from ChatMessage[].
      // If T is expected to be something else, additional mapping mi
ght be needed.
      setStoredValue(messagesWithDates as unknown as T);
      sessionStorage.setItem(key, JSON.stringify(messagesWithDates)
); // Also update sessionStorage with fetched data
      console.log('History fetched from backend and stored.');
    } catch (error) {
      console.error('Failed to fetch history from backend, falling
back to sessionStorage:', error);
      setStoredValue(readValue()); // Fallback to local storage on e
rror
    }
  }, [key, readValue]);

  // Effect to read initial value from backend or sessionStorage on m
ount
  useEffect(() => {
    fetchHistoryFromBackend();
  }, [fetchHistoryFromBackend]);

  // Return a wrapped version of useState's setter function that ...
  // ... persists the new value to sessionStorage.
  const setValue = useCallback((value: T | ((val: T) => T)) => {
    // Prevent build error "window is undefined" but keeps working
    if (typeof window == 'undefined') {
      console.warn(
        `Tried setting sessionStorage key “${key}” even though envir
onment is not a client`,
      );
      return;
    }

    try {
      // Allow value to be a function so we have same API as useState
      const valueToStore = value instanceof Function ? value(storedV
alue) : value;
      // Save state
      setStoredValue(valueToStore);
      // Save to session storage
      window.sessionStorage.setItem(key, JSON.stringify(valueToStore
));
    } catch (error) {
      console.warn(`Error setting sessionStorage key “${key}”:`, er
ror);
    }
  }, [key, storedValue]);
  

  return [storedValue, setValue];
}

export default useSessionStorage;
