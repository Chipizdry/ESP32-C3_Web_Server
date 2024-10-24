

 // Функция для подтверждения переключения режима
 function confirmModeSwitch() {
    var modal = document.getElementById('modeConfirmationModal');
    var mode = modal.dataset.mode;

    // Отправляем запрос на сервер для переключения режима
    fetch('/wifi_mode', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ mode: mode })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            alert('Режим успешно переключен на ' + (mode === 'AP' ? 'Access Point' : 'Station'));
        } else {
            alert('Ошибка при переключении режима.');
        }
    })
    .catch(error => {
        console.error('Ошибка:', error);
    });

    modal.style.display = 'none';
}

// Функция для отмены переключения режима
function cancelModeSwitch() {
    var modal = document.getElementById('modeConfirmationModal');
    modal.style.display = 'none';
}
