"""Security utilities for the Physical AI RAG chatbot API"""

from fastapi import HTTPException, status, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from typing import Optional
import os
import jwt
from datetime import datetime, timedelta
import secrets

# Get API key from environment or generate a default one (for development only)
API_KEY = os.getenv("REACT_APP_PHYSICAL_AI_API_KEY", secrets.token_urlsafe(32))
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

class APIKeyValidator:
    """API key validation utility class"""

    @staticmethod
    def validate_api_key(api_key: str) -> bool:
        """Validate the provided API key"""
        return secrets.compare_digest(api_key, API_KEY)

    @staticmethod
    def generate_api_key() -> str:
        """Generate a new API key (for admin use)"""
        return secrets.token_urlsafe(32)

def verify_api_key(authorization: Optional[str] = None) -> bool:
    """Verify API key from authorization header"""
    if not authorization:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="API key is missing",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Check if it's a Bearer token
    if authorization.startswith("Bearer "):
        token = authorization[7:]
    else:
        # Assume it's just the API key
        token = authorization

    if not APIKeyValidator.validate_api_key(token):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid API key",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return True

async def api_key_auth(request: Request) -> bool:
    """API key authentication dependency"""
    # Try to get API key from header
    auth_header = request.headers.get("Authorization")
    api_key = request.headers.get("X-API-Key")

    # If not in X-API-Key header, try Authorization header
    if not api_key and auth_header:
        return verify_api_key(auth_header)
    elif api_key:
        return verify_api_key(api_key)
    else:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="API key is missing",
            headers={"WWW-Authenticate": "Bearer"},
        )

# JWT token utilities (for user authentication if needed)
def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """Create a JWT access token"""
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)

    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, os.getenv("JWT_SECRET_KEY", "secret"), algorithm=ALGORITHM)
    return encoded_jwt

def verify_access_token(token: str):
    """Verify a JWT access token"""
    try:
        payload = jwt.decode(token, os.getenv("JWT_SECRET_KEY", "secret"), algorithms=[ALGORITHM])
        return payload
    except jwt.ExpiredSignatureError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired",
            headers={"WWW-Authenticate": "Bearer"},
        )
    except jwt.JWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate token",
            headers={"WWW-Authenticate": "Bearer"},
        )