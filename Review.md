# Review gr-rs Florian Rohm 24.7.2020

## Erster Blick
Was ist ein monokularer (?) Graph? Googeln bringt nur explizite paper -> Wird verschoben.

Benutzung scheint in der Readme sehr einfach, Verlinkung zur [README.md](examples/README.md) wäre cool.

## Cargo.toml
Auch wenns nur klein ist, ich hätte mir eine [lib] Sektion gewünscht die zumindest den Namen und den Pfad enthält, auch wenn es der standardweg ist. So weiß ich gleich, dass hier nur eine lib drin ist und nicht auch noch eine executable die dann z.B. eine cli kapselt; soweit ich weiß ein recht häufiges pattern und auch im [latest stable buch](https://doc.rust-lang.org/stable/book/ch12-03-improving-error-handling-and-modularity.html) vorhanden.

Notiz: criterion ist an board, cool! Ich bin gespannt.

Notiz: dependencies sind bis auf kiss3d und nalgebra auf der neuesten Minor Version, cool!

> **Tipp:** ich benutze serayuzgur.crates um mir in vscode die Versionen anzeigen zu lassen, 

## Tests & Code quality
Bevor ich in den code schaue, frage ich gerne mal die tests&code quality was sie von dem code halten:

```cargo test --lib```
        
        failures:
            optimizer::tests::test_all_2d_factors
            optimizer::tests::test_multiple_iterations_2d
            optimizer::tests::test_only_odo2d_factors
            optimizer::tests::test_pos2d_and_odo2d_factors

        test result: FAILED. 40 passed; 4 failed; 5 ignored; 0 measured; 0 filtered out

Und: extrem viel stdout text, dass ich die Testfehler nicht sehe.

> **Tipp:** Main/Master branch bei tests immer sauber halten, wenn die testsuite so schnell ist wie die hier kann man das auch gerne auf einen pre-push hook setzen (hatten wir in einem Javascript Projekt tatsächlich für jeden Branch, hat Wunder gewirkt :-) )

> **Tipp:** Große stdout prints vermeiden. Für debugging Zwecke sowas lieber in Files umleiten und vielleicht sogar in ein Git-repo stecken, dann bekommt man das diff-en geschenkt. In meinem aktuellen Rust-Projekt (numerik simulation) benutze ich etwas recht ähnlich zu:

    use std::io::Write;
    let mut buffer = std::fs::File::create(file_name).expect("creation should not fail");
    writeln!(buffer, "{:.6} ", data).expect("writing should not fail");


```cargo clippy```
Drei warnings in den Standardsettings über unused imports/structs ist imho gar kein Stress!
Selbst wenn ich die pedantic settings auswähle kommt nicht mehr, nice!

## Code
Dafür sind wir doch hier, oder? Ich lasse die Kommentare direkt im code, damit man hier nicht so viel suchen muss kommen die als Kommentare in den code