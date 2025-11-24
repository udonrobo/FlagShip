#include "ThemeController.h"
#include <QDebug>

ThemeController::ThemeController(QObject* parent)
    : QObject{ parent }
{
}

void ThemeController::toggleTheme()
{
    m_theme = (m_theme == Dark) ? Light : Dark;
    emit themeChanged();
}

// --- カラーパレット定義 ---

QColor ThemeController::winBg() const {
    return (m_theme == Dark) ? QColor("#2c313c") : QColor("#f8f9fa");
}

QColor ThemeController::panelBg() const {
    return (m_theme == Dark) ? QColor("#343a45") : QColor("#ffffff");
}

QColor ThemeController::headerBg() const {
    return panelBg();
}

QColor ThemeController::mapBg() const {
    return (m_theme == Dark) ? QColor("#21252b") : QColor("#e9ecef");
}

QColor ThemeController::gridCol() const {
    return (m_theme == Dark) ? QColor("#3a404b") : QColor("#ced4da");
}

QColor ThemeController::textCol() const {
    return (m_theme == Dark) ? QColor("#ffffff") : QColor("#212529");
}

QColor ThemeController::textMuted() const {
    return (m_theme == Dark) ? QColor("#888") : QColor("#6c757d");
}

QColor ThemeController::btnPrimBg() const {
    return (m_theme == Dark) ? QColor("#0d6efd") : QColor("#007bff");
}

QColor ThemeController::btnSecBg() const {
    return QColor("#6c757d");
}

QColor ThemeController::btnTextCol() const {
    return QColor("#ffffff");
}

QColor ThemeController::inpBg() const {
    return (m_theme == Dark) ? QColor("#2c313c") : QColor("#ffffff");
}

QColor ThemeController::inpBorder() const {
    return (m_theme == Dark) ? QColor("#505561") : QColor("#ced4da");
}
