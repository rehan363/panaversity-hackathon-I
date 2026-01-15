import { useState, useEffect, useCallback } from 'react';

/**
 * Custom hook for persisting state to sessionStorage.
 *
 * @param key The key to use in sessionStorage.
 * @param initialValue The initial value to use if no value is found in sessionStorage.
 * @returns A stateful value, and a function to update it.
 */
function useSessionStorage<T>(key: string, initialValue: T): [T, (value: T | ((val: T) => T)) => void] {
  // Get from session storage then
  // parse stored json or return initialValue
  const readValue = useCallback((): T => {
    // Prevent build error "window is undefined" but keep keep working
    if (typeof window === 'undefined') {
      return initialValue;
    }

    try {
      const item = window.sessionStorage.getItem(key);
      return item ? (JSON.parse(item) as T) : initialValue;
    } catch (error) {
      console.warn(`Error reading sessionStorage key “${key}”:`, error);
      return initialValue;
    }
  }, [initialValue, key]);

  // State to store our value
  // Pass initial state function to useState so logic is only executed once
  const [storedValue, setStoredValue] = useState<T>(readValue);

  // Return a wrapped version of useState's setter function that ...
  // ... persists the new value to sessionStorage.
  const setValue = (value: T | ((val: T) => T)) => {
    // Prevent build error "window is undefined" but keeps working
    if (typeof window == 'undefined') {
      console.warn(
        `Tried setting sessionStorage key “${key}” even though environment is not a client`,
      );
    }

    try {
      // Allow value to be a function so we have same API as useState
      const valueToStore = value instanceof Function ? value(storedValue) : value;
      // Save state
      setStoredValue(valueToStore);
      // Save to session storage
      window.sessionStorage.setItem(key, JSON.stringify(valueToStore));
    } catch (error) {
      console.warn(`Error setting sessionStorage key “${key}”:`, error);
    }
  };
  
  useEffect(() => {
    setStoredValue(readValue());
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  return [storedValue, setValue];
}

export default useSessionStorage;
