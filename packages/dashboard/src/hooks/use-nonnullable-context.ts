import React from 'react';

export function useNonNullableContext<T>(context: React.Context<T | null>): T {
  const value = React.useContext(context);
  if (value == null) {
    throw Error('Context is null or undefined');
  }
  return value;
}

export default useNonNullableContext;
