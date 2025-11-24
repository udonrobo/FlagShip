#ifndef THEMECONTROLLER_H
#define THEMECONTROLLER_H

#include <QObject>
#include <QColor>

class ThemeController : public QObject
{
    Q_OBJECT
        // QMLバインディング用プロパティ
        Q_PROPERTY(QColor winBg READ winBg NOTIFY themeChanged)
        Q_PROPERTY(QColor panelBg READ panelBg NOTIFY themeChanged)
        Q_PROPERTY(QColor headerBg READ headerBg NOTIFY themeChanged)
        Q_PROPERTY(QColor mapBg READ mapBg NOTIFY themeChanged)
        Q_PROPERTY(QColor gridCol READ gridCol NOTIFY themeChanged)
        Q_PROPERTY(QColor textCol READ textCol NOTIFY themeChanged)
        Q_PROPERTY(QColor textMuted READ textMuted NOTIFY themeChanged)
        Q_PROPERTY(QColor btnPrimBg READ btnPrimBg NOTIFY themeChanged)
        Q_PROPERTY(QColor btnSecBg READ btnSecBg NOTIFY themeChanged)
        Q_PROPERTY(QColor btnTextCol READ btnTextCol NOTIFY themeChanged)
        Q_PROPERTY(QColor inpBg READ inpBg NOTIFY themeChanged)
        Q_PROPERTY(QColor inpBorder READ inpBorder NOTIFY themeChanged)

public:
    explicit ThemeController(QObject* parent = nullptr);

    enum Theme {
        Dark,
        Light
    };
    Q_ENUM(Theme)

        // 色取得メソッド
        QColor winBg() const;
    QColor panelBg() const;
    QColor headerBg() const;
    QColor mapBg() const;
    QColor gridCol() const;
    QColor textCol() const;
    QColor textMuted() const;
    QColor btnPrimBg() const;
    QColor btnSecBg() const;
    QColor btnTextCol() const;
    QColor inpBg() const;
    QColor inpBorder() const;

public slots:
    void toggleTheme();

signals:
    void themeChanged();

private:
    Theme m_theme = Dark;
};

#endif // THEMECONTROLLER_H
