#!/bin/bash
echo "Очистка кэша Android Studio и Gradle..."

# Очистка Gradle кэша
./gradlew clean
./gradlew --stop

# Удаление .idea и .gradle папок
rm -rf .idea
rm -rf .gradle
rm -rf */build
rm -rf */.gradle

echo "Кэш очищен! Теперь:"
echo "1. Откройте проект в Android Studio"
echo "2. File > Invalidate Caches and Restart > Invalidate and Restart"
echo "3. Подождите полной синхронизации и индексации проекта"
